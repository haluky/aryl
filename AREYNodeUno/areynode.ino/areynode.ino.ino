/*
  LoRa Duplex communication
  Sends a message every half second, and polls continually
  for new incoming messages. Implements a one-byte addressing scheme,
  with 0xFF as the broadcast address.
  Uses readString() from Stream class to read payload. The Stream class'
  timeout may affect other functuons, like the radio's callback. For an
  created 28 April 2017
  by Tom Igoe
*/
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include "TSL2561.h"

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#define DHTPIN            4         // Pin which is connected to the DHT sensor.
#define DHTTYPE           DHT11     // DHT 11 
DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

#include "Timer.h"
int uptime = 0;
Timer t;

TSL2561 tsl(TSL2561_ADDR_FLOAT);


int redpin = 1; // select the pin for the red LED
int greenpin = 2; // select the pin for the green LED

const int csPin = 10;          // LoRa radio chip select
const int resetPin = 9;       // LoRa radio reset
const int irqPin = 3;         // change for your board; must be a hardware interrupt pin

String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xAA;     // address of this device
byte destination = 0xBB;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends

int lightLevel=0;


extern "C" {
  extern void resetZero();
}


void setup() {
  Serial.begin(9600);                   // initialize serial
  while (!Serial);
 
  Serial.println("LoRa Duplex");
  ///dhtttt
  /////////////////
  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");
  Serial.println("------------------------------------");
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");
  Serial.println("------------------------------------");
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;

  //////////////////////////
  //////////////////////////


  /////////////

  tsl.setGain(TSL2561_GAIN_16X);
  tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS); // shortest integration time (bright light)

  //////


  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(433E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
  pinMode (redpin, OUTPUT);
  pinMode (greenpin, OUTPUT);
  Serial.println("LoRa init succeeded.");

  t.every(1000, taskX);

}
String message="";
void loop() {
  onReceive(LoRa.parsePacket());

  t.update();



  
  /*
    if (millis() - lastSendTime > interval)
    {
     sendMessage(message);

     Serial.println("Sending " + message);
     lastSendTime = millis();            // timestamp the message
     interval = random(1000) + 2000;    // 2-3 seconds

    }*/
  // parse for a packet, and call onReceive with the result:


}
int val;

void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID



}
  
String incoming = "0";
String rssiX="";

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return


  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

 
if (recipient == localAddress)
  {

incoming ="";
  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }
 }
  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress)
  {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  rssiX = String(LoRa.packetSnr());
  Serial.println();
  
  lightLevel= incoming.toInt();
  
  
      analogWrite (greenpin, lightLevel);

  
  if (incoming == "AIX")

  {
    analogWrite (greenpin, 255);
    analogWrite (redpin, 0);

    /*for (val = 255; val> 0; val--)
        {
        analogWrite (greenpin, val);
        analogWrite (redpin, 255-val);
        delay (15);
      }
    */
  }
  else if ( incoming == "HAR")
  {
    analogWrite (redpin, 255);
    analogWrite (greenpin, 0);

    /*
      for (val = 0; val <255; val++)
        {
        analogWrite (greenpin, val);
        analogWrite (redpin, 255-val);
    *///   delay (15);

  }
}
void(* resetFunc) (void) = 0; //declare reset function @ address 0


void taskX()
{
  //onReceive(LoRa.parsePacket());
  uptime = millis() / 1000;

   if (millis() - lastSendTime > interval) {
   
   
  uint16_t x = tsl.getLuminosity(TSL2561_VISIBLE);
  //Serial.println(x, DEC);
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;

  sensors_event_t event;
  dht.temperature().getEvent(&event);
  int tempTemp = event.temperature;
  String lux = String(tsl.calculateLux(full, ir));

  String temperature = String(tempTemp);



  message = incoming + "-" + temperature ; // send a message

   
   
   
   
   
   
   
   
   
   
   
   
    sendMessage(message);
    Serial.println("Sending " + message);
    lastSendTime = millis();            // timestamp the message
    interval = random(20000) + 1000;    // 2-3 seconds
  }

  


  if (uptime > 6000)
    resetFunc();  //call reset


}



