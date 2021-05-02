#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "DHT.h"
#include "time.h"
#include "credentials.h"
extern "C" {
#include "libb64/cdecode.h"
}


#define DHTTYPE DHT22   // DHT 22  humidity temp sensor
#define DHTPIN 13     
#define SOIL_PIN A0

DHT dht(DHTPIN, DHTTYPE); //initialize DHT sensor

//Imported from credentials.h, provide your own :)
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWD;
const char* awsEndpoint = AWS_END_POINT;
const String certificatePemCrt =PEM_CERT; 
const String privatePemKey = PEM_KEY;
const String caPemCrt = CA_CERT;

WiFiClientSecure wiFiClient;
void msgReceived(char* topic, byte* payload, unsigned int len);
PubSubClient pubSubClient(awsEndpoint, 8883, msgReceived, wiFiClient); 

void setup() {
  Serial.begin(74880); Serial.println();
  Serial.println("Begin");

  //start temp/humidity reading
  dht.begin();
  
  //make initial connection to wifi
  Serial.print("Connecting to "); Serial.print(ssid);
  WiFi.begin(ssid, password);
  WiFi.waitForConnectResult();
  Serial.print(", WiFi connected, IP address: "); Serial.println(WiFi.localIP());

  // get current time for certs
  configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time-a-g.nist.gov");
  getCurrentTime();
  
  uint8_t binaryCert[certificatePemCrt.length() * 3 / 4];
  int len = b64decode(certificatePemCrt, binaryCert);
  wiFiClient.setCertificate(binaryCert, len);
  
  uint8_t binaryPrivate[privatePemKey.length() * 3 / 4];
  len = b64decode(privatePemKey, binaryPrivate);
  wiFiClient.setPrivateKey(binaryPrivate, len);

  uint8_t binaryCA[caPemCrt.length() * 3 / 4];
  len = b64decode(caPemCrt, binaryCA);
  wiFiClient.setCACert(binaryCA, len);
}

unsigned long lastTransmit;
int soilMoisture, badReadCount = 0;

void loop() {
  //check connection to AWS
  pubSubCheckConnect();

  //pull soil moisture reading
  soilMoisture = analogRead(SOIL_PIN);

  //grab humidity + temperature
  float humidity = dht.readHumidity();
  float fahrenheit = dht.readTemperature(true);

  //check to see if reads failed
  if (isnan(humidity) || isnan(fahrenheit)){
    //bad read from sensor, re-attempt
    return;
  }

  //calculate heat index using humidity and temperature
  float heatIndex = dht.computeHeatIndex(fahrenheit, humidity);
  
  //create our JSON
  StaticJsonDocument<200> doc;

  doc["device"] = "ESP2866-Left";
  doc["temperature"] = fahrenheit;
  doc["humidity"] = humidity;
  doc["heatIndex"] = heatIndex;
  doc["soilMoisture"] = soilMoisture;
  doc["timestamp"] = getCurrentTime();

  char buffer[256];
  size_t payloadSize = serializeJson(doc, buffer);
  pubSubClient.publish("outTopic", buffer);
  Serial.print("Published: "); Serial.println(buffer);
  delay(20000); 
}


//boilerplate for later use
void msgReceived(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on "); 
  Serial.print(topic); 
}

//aws connection method
void pubSubCheckConnect() {
  if ( ! pubSubClient.connected()) {
    Serial.print("PubSubClient connecting to: "); 
    Serial.print(awsEndpoint);
    while ( ! pubSubClient.connected()) {
      Serial.print(".");
      pubSubClient.connect("gardenSensor");
    }
    Serial.println(" connected");
    pubSubClient.subscribe("inTopic");
  }
  pubSubClient.loop();
}

//method needed for certs
int b64decode(String b64Text, uint8_t* output) {
  base64_decodestate s;
  base64_init_decodestate(&s);
  int cnt = base64_decode_block(b64Text.c_str(), b64Text.length(), (char*)output, &s);
  return cnt;
}

//grab current time
unsigned long getCurrentTime() {
  time_t now = time(nullptr);
  struct tm timeinfo;
  time(&now);
  return now;
}
