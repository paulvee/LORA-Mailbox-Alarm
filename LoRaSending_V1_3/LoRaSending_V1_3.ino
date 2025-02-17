// Compile with board: LOLIN(WEMOS) D1, D2 & Mini
/*
  This sketch is used to signal a receiver that mail is detected in the mailbox.
  Upon detection, it will be woken-up from a deep sleep mode with a reset and send an 
  "I have mail" message to the receiver, and then waits for an acknowledgement. 
  If that is not received, it sends the "I have mail" message a few more times. 
  When the acknowledgement is received the sketch sends 
  the actual LiPo cell voltage to the receiver so it can set an alarm when the cell 
  voltage is deemed too low and the cell can be replaced. 
  After this cycle, the ESP8266 goes back to sleep to conserve power.
*/

#include <SoftwareSerial.h> // serial to communicate with the LORA board

//  cross reference port defines between models (Arduino Nano & ESP8266)
// #define D0  16  // wake from sleep, Flash, no sw pull_up, use 10K resistor
// #define D1  5   // i2c bus SCL
// #define D2  4   // i2c bus SDA
// #define D3  0   // 10K pull-up
// #define D4  2   // 10K pull-up, same as LED_BUILTIN, but inverted logic
// #define D5  14  // spi bus SCK
// #define D6  12  // spi bus MISO
// #define D7  13  // spi bus MOSI
// #define D8  15  // spi bus SS/CS 10K pull-down(do not use, misbehaves with fw download)
// #define D9  3   // RX0 : Not available on WEMOS D1 mini
// #define D10 1   // TX0 : Not available on WEMOS D1 mini
//
const String FW_VERSION = "V1.3"; 

// Letterbox receiver board port connections
#define M0 D1   // LORA mode input
#define M1 D2   // LORA mode input
#define AUX D3  // LORA status output
#define LED D4  // built-in LED on 8266 module
#define TXD2 D6 // Software serial for LORA
#define RXD2 D5 // Software serial for LORA
#define TRIG_EN D7 // enable pin for the trigger circuit

SoftwareSerial Serial2(RXD2, TXD2);  // to communicate with the LORA board

byte receivedCode = 0; // byte buffer for the serial receiver
String receivedString; // used to build a string from the incoming bytes
char buff[10]; // holding the battery voltage string
bool lora_ack = false; // register the ack from the receiver to the msg
int retry = 0; // retry the sending until we have an ack
int max_retries = 3;  // re-send the msg until we get an ack
// this var will be stored in RTC memory (RAM that persists across restarts, but not power outs)
uint32_t low_bat_alarm = 0;



void setup() {
  // all code will run in setup

  // Activate this line in case we ended-up with the alarm somehow already set
  //ESP.rtcUserMemoryWrite(0, &low_bat_alarm, sizeof(low_bat_alarm)); 

  pinMode(TRIG_EN, OUTPUT); // The alarm trigger circuit enable
  digitalWrite(TRIG_EN, LOW); // Disable the trigger circuit during setup

  // for debugging
  pinMode(LED, OUTPUT); // the built-in LED on the ESP8266 module
  digitalWrite(LED, HIGH); // Make sure it's off to save power (reversed polarity)
  // comment out when deploying

  pinMode(M0, OUTPUT); // these 2 ports have a very light pull_up from the LORA board
  pinMode(M1, OUTPUT);
  pinMode(AUX, INPUT); // ESP8266 already has a pull_up on this port. When using another port, use pull_up
  digitalWrite(M0, HIGH); // put the LoRa to sleep during the booting process
  digitalWrite(M1, HIGH); // this is also the command format
  delay(1000); // really shut the LoRa board down so it does not transmit and activates the trigger circuit to reset again

  // for debugging and testing
  Serial.begin(9600);
  while (!Serial) delay(500);
  delay(5000); // we need this to catch the beginning...
  Serial.print("\n\r\n\rStarting Letterbox Sender ");
  Serial.println(FW_VERSION);
  Serial.printf("Reason for reboot: %s\n", ESP.getResetReason().c_str());

  Serial.print("Bootmode = "); Serial.println(bootMode);
  // end of debugging and testing code, can be deactivated when deployed, together
  // with all the other Serial.print() statements

  // read the low_bat_alarm status from RTC memory (RAM that persists across restarts, but not power outs)
  if(!ESP.rtcUserMemoryRead(0, &low_bat_alarm, sizeof(low_bat_alarm))) {
    Serial.println("RTC read failed!");
    while(1)
      yield();
  }

  // Read the battery voltage
  // Also use that as a flag to prevent execution of the normal procedure because when the 
  // battery alarm is active, it will otherwise continue to reboot the system in an endless loop
  // draining the battery even more
  int in0 = analogRead(A0);
  //Serial.print("analog read = "); Serial.println(in0);
  // conversion from binary to voltage
  // vref=1.0, bits=1024, ADC value-7bits offset (value with shorted A0 input), input attenuator calculation, calibration value=0.982
  float battV = 1.0 / 1024 * (in0 - 7) * ((180000+220000+100000)/100000) * 0.982; // update the 0.982 correction factor based on your hardware
  Serial.print("Cell voltage: "); Serial.println(battV,4);
  dtostrf(battV, 4, 3, buff);
  if (battV <= 3.21){ // the low batt trigger is active and will continue to reset the ESP
    Serial.println("low batt alarm active");
    low_bat_alarm = 1; // if we have an alarm already, skip the reboots from now on
    // write the low_bat_alarm status to RTC memory to save it
    // Idea borrowed from here: https://arduino.stackexchange.com/questions/91580/esp8266-rtc-memory-for-bootcount
    if(!ESP.rtcUserMemoryWrite(0, &low_bat_alarm, sizeof(low_bat_alarm))) {
      Serial.println("RTC write failed!");
      while(1)
        yield();
    }
  }else{ // no cell voltage alarm yet
    // always update the low_bat_alarm status in RTC memory when the cell voltage is still OK
    low_bat_alarm = 0; // reset the flag and save it
    if(!ESP.rtcUserMemoryWrite(0, &low_bat_alarm, sizeof(low_bat_alarm))) {
      Serial.println("RTC write failed!");
      while(1)
        yield();
    }
  }

  Serial.print("low_bat_alarm = "); Serial.println(low_bat_alarm);

  if (low_bat_alarm == 0){ // cell voltage is still OK, run normally
    Serial2.begin(9600, SWSERIAL_8N1, RXD2, TXD2); // setup sw serial to communicate with the LORA board
    // setup the LORA board ports to accept programming of the parameters
    // it is still in sleep/command mode
    Serial.println("Sending initialization to the LoRa board");
    // 0xCO = start byte, send 5 more bytes with the parameters
    Serial.println("adrs=0, Ser=8N1, 9600bd, Air=2.4Kbps, chan=23, Pwr=20dBm");
    byte data[] = { 0xC0, 0x0, 0x0, 0x1a, 0x17, 0x44 }; // default
    // send the bytes one-by-one
    for (int i = 0; i < sizeof(data); i++) {
      Serial2.write(data[i]);
      //Serial.print(data[i], HEX);
      //Serial.print(" ");
    }
    //Serial.println(); // terminate the string of bytes

    // AUX = busy status; low during self test and initialization and high during serial transmission
    while (digitalRead(AUX) == LOW)
      ; // must wait for the AUX output to become high

    // configure the LORA board for communication and set it to General Mode
    digitalWrite(M0, LOW);
    digitalWrite(M1, LOW);
    delay(1000);
    Serial2.flush(); // flush the serial buffer

    Serial.println("Setup is done");

    // this message will be sent even after the very first power up so it will be a false alarm
    // but we'll know if there has been a power issue.
    Serial2.print("I have mail..."); // transmitting via LORa
    Serial.println("sending I have mail");
    delay(3000); // LORA is pretty slow, give the receiver some time to respond

    // wait for an acknowledgement from the receiver, if not coming, try again
    while (!lora_ack && retry <= max_retries){
      Serial2.flush(); // flush the buffer from any stuck remains
      receivedString="";
      delay(3000); // LoRa is pretty slow, give the receiver some time to respond

      // see if there is serial data coming in, hopefully with an acknowledgement
      while (Serial2.available() > 0) {
        receivedCode = Serial2.read();
        //Serial.println(receivedCode, HEX);
        receivedString += char(receivedCode); // build the string by converting bytes to char
      }
      //Serial.println("we have incoming LoRa serial data");
      Serial.print("Received string: "); Serial.println(receivedString);
      if (receivedString == "ack\r\n"){ // need to include the cr and lf
        Serial.println("ack received");
        break;
      }else{
        Serial.println("bummer, no ACK");
        Serial2.print("I have mail..."); // transmitting again via LORA 
        Serial.println("sending I have mail");
        Serial.print("number of tries: "); Serial.println(retry);
        retry++;
      }
    }

    // send the battery voltage as a return for the ACK received from the receiver
    // the receiver will turn on a warning LED if the voltage is too low, determined by the receiver
    dtostrf(battV, 4, 3, buff);
    Serial.printf("Sending battery voltage: %s\n", buff);
    Serial2.print(buff);
    delay(2000); // make sure we get the message out before we go to sleep
  } // end of the testing for the presence of the voltage alarm


  // Put the LoRa module in sleep mode, the current goes down to 4uA.
  // This is to force the pins to float high, although there is a light pull-up on the board.
  // When we put the ESP also to sleep, those LoRa module pull-ups should take over anyway.
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);

  // ===== for debugging & testing
  Serial.println("Entering deep sleep mode.\r\n");
  digitalWrite(LED, LOW); // Turn the on board LED on to signal the entering of the deep sleep mode (reversed polarity)
  delay(1000); 
  digitalWrite(LED, HIGH); // Make sure it's off to save power (reversed polarity)  
   
  // we're done so we can arm the distress trigger circuit so it's active while we're sleeping
  Serial.println("arming the alarm trigger circuit");
  pinMode(TRIG_EN, INPUT); // Make the port high impedance so the pull_up resistor is taking over

  ESP.deepSleep(0); // night-night, sleep tight.
  // In deep sleep mode, the ESP8266 wakes up when there is a negative spike on the RST pin
  // in this case, it will be the letterbox opening switch or a cell voltage distress warning
  // after waking-up, the setup() code will run from the start
}


void loop() {
// should not get here, not used.
}

// end of code