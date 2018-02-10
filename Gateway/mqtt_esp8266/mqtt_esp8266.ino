#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Update these with values suitable for your network.

const char* ssid = "sungerbob";
const char* password = "halukbinreyhan";
const char* mqtt_server = "areyhome.dynu.com";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;



#include <SPI.h>              // include libraries
#include <LoRa.h>
#include "Timer.h"

const int csPin = D8;          // LoRa radio chip select
const int resetPin = D0;       // LoRa radio reset
const int irqPin = D1;         // change for your board; must be a hardware interrupt pin

String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends




///
/// LoRa Stage xy1 0035
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
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
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
    if (client.connect("AREYGATE_1")) {
      Serial.println("connected");
      // Once connected, publish an announcement...

      
      client.publish("outTopic", "hello world");


      // ... and resubscribe
      client.subscribe("CORE/ARYN");
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


void sendMessage(String outgoing, short adress) {
//digitalWrite(BUILTIN_LED,HIGH);
  
  LoRa.beginPacket();                   // start packet
  LoRa.write(adress);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID

//digitalWrite(BUILTIN_LED,LOW);

}

int resetleyiciSayac=0;

int totalTime=0;
int resetMe=0;
int uptime=0;

void taskX() 
{
  //onReceive(LoRa.parsePacket());
  uptime=millis()/1000;
 // totalTime=millis()/1000;
  
 //Serial.println(uptime);

 
  resetleyiciSayac+=1;
 
  Serial.println(resetleyiciSayac);

  if(resetleyiciSayac==3000)
  ESP.restart();
   client.publish("CORE/applications/51OSB/STATE", "im Alive!");

 
  if(totalTime>5)
  totalTime=0;

  if(uptime>1000)
  ESP.restart();
  
}