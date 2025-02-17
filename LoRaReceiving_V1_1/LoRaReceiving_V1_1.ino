// compile for board WEMOS D1 mini
/*
  This sketch is used to receive the "I have mail" message from the mailbox 
  sender, when it has detected incoming mail. Upon receiving the message, this
  sketch sends an acknowledgement, and when the sender receives that, it will 
  send the LiPo cell voltage and then goes back to sleep. Upon receiving the 
  mail message, an LED is turned on, and a beeper sounds an alarm for a few times.
  The cell voltage is analyzed, and when deemed too low, turns on an LED.
  A button is used to reset both signals.
*/
#include <SoftwareSerial.h> // serial to communicate with the LORA board
#include <Ticker.h>       // For OS.watchdog

const String FW_VERSION = "V1.1"; 
//
//  cross reference, defines between models (ESP8266 -> Arduino Nano)
// #define D0  GPIO 16  // wake from sleep, no INT,PWM or i2c, Flash, no pull_up, use 10K resistor
// #define D1  GPIO 5   // i2c bus SCL (i2c)
// #define D2  GPIO 4   // i2c bus SDA (i2c)
// #define D3  GPIO 0   // 10K pull-up (boot fails if pulled low)
// #define D4  GPIO 2   // 10K pull-up, same as LED_BUILTIN, but inverted logic
// #define D5  GPIO 14  // spi bus SCLK
// #define D6  GPIO 12  // spi bus MISO
// #define D7  GPIO 13  // spi bus MOSI
// #define D8  GPIO 15  // spi bus SS/CS 10K pull-down(boot fails if pulled hi, misbehaves with fw download)
// #define D9  GPIO 3   // RX0 (high at boot)
// #define D10 GPIO 1   // TX0 (high at boot, boot fails if pulled low)
// #define A0  ADC 0    // analog input

// LoRa board connections
#define M0 D1   // mode input
#define M1 D2   // mode input
#define AUX D3  // status output
//#define LED_BUILTIN D4  // already defined: blue built-in LED => reversed polarity!
#define TXD2 D6
#define RXD2 D5
#define BUTTON A0 // button to stop the beeper
#define MAILLED D0  // we have mail 
#define BEEPER D8 // the beeper
#define BATLED D7 // low battery voltage


SoftwareSerial Serial2(RXD2, TXD2);  // to communicate with the LORA board

byte receivedCode = 0;
String receivedString;
bool ack = false;
float battVoltage;
bool beeper = false;
int maxBeepCount = 5; // the number of times we beep after the "we have mail" message
int beepCounter = 0; // the running total
bool mail = false; // set when we have mail to avoid false positives after booting


void setup() {
    Serial.begin(9600);
  while (!Serial) delay(500);
  delay(5000); // we need this to catch the beginning...
  Serial.print("\n\r\n\rStarting Letterbox Receiver ");
  Serial.println(FW_VERSION);
  Serial.printf("Reason for reboot: %s\n", ESP.getResetReason().c_str());

  Serial2.begin(9600, SWSERIAL_8N1, RXD2, TXD2); // serial to communicate with the LORA board
  
  pinMode(M0, OUTPUT); // these 2 ports have a very light pull_up
  pinMode(M1, OUTPUT);
  pinMode(AUX, INPUT); // ESP8266 has pull_up on this port, otherwise, use pull_up
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BATLED, OUTPUT); // low battery voltage alarm
  digitalWrite(BATLED, LOW); // off 
  pinMode(MAILLED, OUTPUT); // we have mail
  digitalWrite(MAILLED, LOW); // off  
  pinMode(BEEPER, OUTPUT); // beeper
  digitalWrite(BEEPER, LOW); 
  // blink the on-board LED to signal the bootup (has inverted logic)
  digitalWrite(LED_BUILTIN, LOW); // on => inverted logic
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH); // off

  //set LoRa chip to programming mode
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  delay(10); // >2ms

  // 0xCO = start byte, expect 5 more bytes,
  Serial.println("adrs=0, Ser=8N1, 9600bd, Air=2.4Kbps, chan=23, Pwr=20dBm");
  byte data[] = { 0xC0, 0x0, 0x0, 0x1a, 0x17, 0x44 }; // default
  for (int i = 0; i < sizeof(data); i++) {
    Serial2.write(data[i]);
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }

  // AUX = busy status; low during self test and initialization and high during serial transmission
  while (digitalRead(AUX) == LOW)
    ; // must wait for the status to become high
  Serial.println("\n\ractivate the Lora Board...");
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  delay(100); // >2ms
  Serial2.flush();
  Serial.println("Setup finished, running...");
}

void loop() {

  Serial2.flush();
  receivedString="";

  delay(1000);  // this is important to get the software serial.read working properly after a reboot
                // it also slows the loop time to 1sec

  // see if there is serial data coming in
  if (Serial2.available() > 0) {
    while (Serial2.available() > 0) {
      receivedCode = Serial2.read();
      //Serial.println(receivedCode, HEX);
      if ((' ' <= receivedCode) && (receivedCode <= '~')) // everything else is rubbish
        receivedString += char(receivedCode); // build the string by converting bytes to char
    }

    if (receivedString.length() > 4) { // voltage = 5; smaller must be rubbish
      Serial.print("Received string : "); Serial.println(receivedString);
      if (receivedString == "I have mail..."){//\r\n"){
        Serial.println("We have mail!");
        flash_led();
        Serial.println("sending ack to confirm the receipt");
        Serial2.println("ack"); // confirm that we have received the message
        mail = true;
        beeper = true;
        beepCounter = 0;
        receivedString="";
        Serial2.flush();
      }else{
        if (mail == true) {
          battVoltage = receivedString.toFloat();
          Serial.print("battery voltage : ");
          Serial.print(battVoltage,3);
          Serial.println(" V");
          checkVolt();
        }
      }
    }
  }

  //Serial.println(analogRead(A0));
  if (analogRead(A0) > 100){ // normally about 16-17b, raised to 1024 (3V3) by button
      beeper = false;
      mail = false;
      digitalWrite(BATLED, LOW); // Turn the low battery voltage warning LED off
      //Serial.println("stopping beeper");
  }

  // continue to sound the "we have mail" alarm until it is acknowledged
  if (mail == true){
    beep();
    flash_led();
  }
}


void flash_led(){ // blue LED
  if (beeper){
    digitalWrite(MAILLED, HIGH); // on
    digitalWrite(LED_BUILTIN, LOW);
    delay(1);
    digitalWrite(MAILLED, LOW); // off
    digitalWrite(LED_BUILTIN, HIGH);
  }
}


void beep(){
  if (beeper && (beepCounter < maxBeepCount)){
    digitalWrite(BEEPER, HIGH);
    delay(200);
    digitalWrite(BEEPER, LOW); 
    beepCounter ++;
  }
}


void checkVolt(){
  if (battVoltage <= 3.21){ // The cell is getting low and needs to be replaced
    Serial.println("LED batt low"); // red LED
    digitalWrite(BATLED, HIGH); // on
  } else{
    Serial.println("batt ok");
    digitalWrite(BATLED, LOW); // Turn the warning LED off
  }
  battVoltage = 0;
}
