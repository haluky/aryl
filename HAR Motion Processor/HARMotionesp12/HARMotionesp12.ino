#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "ACS712.h"

// Update these with values suitable for your network.
#include <dht11.h> // dht11 kütüphanesini ekliyoruz.
#define DHT11PIN 10 // DHT11PIN olarak Dijital 2"yi belirliyoruz.
dht11 DHT11;

const char* ssid = "sungerbob";
const char* password = "halukbinreyhan";
const char* mqtt_server = "areyhome.dynu.com";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
int greenpin = 10; // select the pin for the green LED

//acs712
const int sensorIn = A0;
int mVperAmp = 66; // use 100 for 20A Module and 66 for 30A Module
ACS712 sensor(ACS712_30A, A0);


double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;


#include <SPI.h>              // include libraries
#include <LoRa.h>
#include "Timer.h"

const int csPin = D8;          // LoRa radio chip select
const int resetPin = D0;       // LoRa radio reset
const int irqPin = D1;         // change for your board; must be a hardware interrupt pin

String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xCC;     // address of this device
byte destination = 0xBB;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends



//HCSR STAGEf

int inputPin = D2;               // choose the input pin (for PIR sensor)
int pirState = LOW;             // we start, assuming no motion detected



///
/// LoRa Stage
///




void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return



  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  int sender = LoRa.read();            // sender address
  int incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
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
  Serial.println();


if(incoming=="ONX")
{
  Serial.println("okeyyy");
      analogWrite (greenpin, 255);

}
 else if(incoming=="OFX")
{

  Serial.println("dddddddokeyyy");
      analogWrite (greenpin, 1);

}

  String bbKing = "ARN0x"+ String(sender, HEX)+"_"+incoming+ String(LoRa.packetRssi());

    char deviceLinkX[50]; 
    bbKing.toCharArray(deviceLinkX,50); 


  Serial.println(sender);  
  Serial.println("Sending to CORE");

     client.publish("CORE/ARN",deviceLinkX);


  
}

String msgString = "";String msgString2 = ""; String firstThree="";















Timer t;

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  pinMode(inputPin, INPUT);     // declare sensor as input
  pinMode (greenpin, OUTPUT);

  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  sensor.calibrate();
 DHT11.attach(D2);

}


void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());



   LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
 
   if (!LoRa.begin(433E6)) {             // initialize ratio at 915 MHz
     Serial.println("LoRa init failed. Check your connections.");
     while (true);                       // if failed, do nothing
   }
 
   Serial.println("LoRa init succeeded.");

   t.every(1000, taskX);
}





void callback(char* topic, byte* payload, unsigned int length) {
 Serial.print("Message arrived [");
 Serial.print(topic);
 Serial.print("] ");
 for (int i=0;i<length;i++) {
  char receivedChar = (char)payload[i];
  Serial.print(receivedChar);
 }

 
 char buffer[length];
 for (int i = 0; i < length; i++)
 {
   msgString += String((char)payload[i]);
   msgString2 += String((char)payload[i]);
 }



 String payloadData = buffer;
 Serial.println(msgString);
 msgString2= msgString;

 firstThree = msgString.substring(0,4);
 Serial.println(firstThree);

 Serial.println("----------------------");
 String lastThree = msgString2.substring(5,8);
 Serial.println(lastThree);
/*
 
 firstThree.toCharArray(destinationAddX,50);
 */

char destinationAddX[50]; 
firstThree.toCharArray(destinationAddX,50);
long myNum = atol(destinationAddX);
short firstThreeShorted = myNum;




sendMessage(lastThree,firstThreeShorted);
 
 msgString="";  msgString2=""; Serial.println();
}




void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ARN Har Motion")) {
      Serial.println("connected");
      // Once connected, publish an announcement...

      
      client.publish("outTopic", "hello world");


      // ... and resubscribe

      client.subscribe("CORE/HAR");
      client.subscribe("CORE\HAR");


    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}






 
void loop()
{
  
  
  
 if (!client.connected()) {
  reconnect();
 }
 client.loop();
 t.update();



  onReceive(LoRa.parsePacket());
}


void sendMessage(String outgoing, byte adress) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

int resetleyiciSayac=0;

int totalTime=0;
int resetMe=0;
int uptime=0;
int val = 0;                    // variable for reading the pin status


int sayacX=0;
void taskX() 
{
  //onReceive(LoRa.parsePacket());
  uptime=millis()/1000;
 // totalTime=millis()/1000;
  
 //Serial.println(uptime);




//GroupID

val = digitalRead(inputPin);  // read input value
  if (val == HIGH) {            // check if the input is HIGH
 //   digitalWrite(BUILTIN_LED, HIGH);  // turn LED ON
    if (pirState == LOW) {
      // we have just turned on



      sendMessage("ARN51OSBA0",0xBB);


      Serial.println("Motion detected!");
      // We only want to print on the output change, not state
      pirState = HIGH;
    }
  } else {
 //   digitalWrite(BUILTIN_LED, LOW); // turn LED OFF
    if (pirState == HIGH){
      // we have just turned of
      Serial.println("Motion ended!");
      // We only want to print on the output change, not state
      pirState = LOW;
    }
  }



 int chk = DHT11.read();
 
 Serial.print("Read sensor: ");
 switch (chk)
 {
 case 0: Serial.println("OK"); break;
 case -1: Serial.println("Checksum error"); break;
 case -2: Serial.println("Time out error"); break;
 default: Serial.println("Unknown error"); break;
 }
 
 Serial.print("Humidity (%): ");   //Nem yüzdesini ekrana yazdırıyoruz
 Serial.println((int)DHT11.humidity, DEC);
 
 Serial.print("Temperature (°C): ");   //Sıcaklığı santigrad olarak ekrana yazdırıyoruz
 Serial.println((int)DHT11.temperature, DEC);
 Serial.println(DHT11.temperature);
 
 Serial.print("Dew Point (°C): ");    //Sıcaklık aynı nem oranında bu sıcaklık değerine düşerse kar,yağmur,sis,çiğ bilimum doğa olayları görülür.
 Serial.println(DHT11.dewPoint(), DEC);
 
 Serial.print("Dew PointFast (°C): ");
 Serial.println(DHT11.dewPointFast(), DEC);
 









/*
  float U = 230;

  // To measure current we need to know the frequency of current
  // By default 50Hz is used, but you can specify own, if necessary
  float I = sensor.getCurrentAC();

  // To calculate the power we need voltage multiplied by current
  float P = U * I;

  Serial.println(String("P = ") + P + " Watts");

 
 Voltage = getVPP();
 VRMS = (Voltage/2.0) *0.707; 
 AmpsRMS = (VRMS * 1000)/mVperAmp;
 Serial.print(AmpsRMS);
 Serial.println(" Amps RMS");






*/

 
  resetleyiciSayac+=1;
  sayacX++;
String fd = String((int)DHT11.temperature, DEC);
if(sayacX==30)
{
sayacX=0;

      sendMessage(fd,0xBB);


}



  Serial.println(resetleyiciSayac);

  if(resetleyiciSayac==3000)
  ESP.restart();
   client.publish("CORE/applications/51OSB/STATE", "im Alive!");

 
  if(totalTime>5)
  totalTime=0;

  if(uptime>1000)
  ESP.restart();
  
}



float getVPP()
{
  float result;
  
  int readValue;             //value read from the sensor
  int maxValue = 0;          // store max value here
  int minValue = 1024;          // store min value here
  
   uint32_t start_time = millis();
   while((millis()-start_time) < 1000) //sample for 1 Sec
   {
       readValue = analogRead(sensorIn);
       // see if you have a new maxValue
       if (readValue > maxValue) 
       {
           /*record the maximum sensor value*/
           maxValue = readValue;
       }
       if (readValue < minValue) 
       {
           /*record the maximum sensor value*/
           minValue = readValue;
       }
   }
   
   // Subtract min from max
   result = ((maxValue - minValue) * 5.0)/1024.0;
      
   return result;
 }