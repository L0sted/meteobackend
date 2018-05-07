//backend
#include <BH1750.h>
#include <BMP280.h>
#include <Wire.h>
#include <NTPClient.h> 
#include <WiFiUdp.h> 
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include "DHTesp.h"
#include <DallasTemperature.h>
#include <OneWire.h>
#include "../wifi.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

float inTemp,humid,extTemp;
double bmpTemp,pressure,altitude,lux;

const short ds18b20 = 4, bmpsda = 5, bmpscl = 16, lightSCL = 14, lightSDA = 12, dhtpin = 0;

BMP280 bmp;
#define P0 1013.25

DHTesp dht;
#define ONE_WIRE_BUS 2  // DS18B20 pin
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, "192.168.100.102", 1883);
Adafruit_MQTT_Publish extTempMQTT = Adafruit_MQTT_Publish(&mqtt, "externalTemp");
Adafruit_MQTT_Publish inTempMQTT = Adafruit_MQTT_Publish(&mqtt, "inTemp");
Adafruit_MQTT_Publish humidMQTT = Adafruit_MQTT_Publish(&mqtt, "humid");
Adafruit_MQTT_Publish bmpTempMQTT = Adafruit_MQTT_Publish(&mqtt, "bmpTemp");
Adafruit_MQTT_Publish pressureMQTT = Adafruit_MQTT_Publish(&mqtt, "pressure");
Adafruit_MQTT_Publish altitudeMQTT = Adafruit_MQTT_Publish(&mqtt, "altitude");

BH1750 lightMeter;

void setup(){
  Serial.begin(115200);
  //==WIFI CONNECT==
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("Connected to " + String(ssid) + "; IP address: " + WiFi.localIP());
  MDNS.begin("esp8266-backend");
  dht.setup(dhtpin);
  //==BMP INIT==
  if(!bmp.begin(bmpsda,bmpscl)){
      Serial.println("BMP init failed!");
  }
  else {
    Serial.println("BMP init success!");  
    bmp.setOversampling(4);
  }
  
  Wire.begin(lightSCL, lightSDA);
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))
    Serial.println("lightMeter error!");
  Serial.println("Go!");
}

void loop(){
  // Serial.println("DHT");
  getAccurateDHT();
  // Serial.println("DS18B20");
  getDS18();
  // Serial.println("BMP280");
  getBMP();
  // Serial.println("MQTT");
  MQTT_loop();
}

void MQTT_loop() {
  MQTT_connect();

  if (! extTempMQTT.publish(extTemp)) {
    Serial.println(F("Failed"));
  }
  if (! inTempMQTT.publish(inTemp)) {
    Serial.println(F("Failed"));
  }
  if (! humidMQTT.publish(humid)) {
    Serial.println(F("Failed"));
  }
  if (! bmpTempMQTT.publish(bmpTemp)) {
    Serial.println(F("Failed"));
  }
  if (! pressureMQTT.publish(pressure)) {
    Serial.println(F("Failed"));
  }
  if (! altitudeMQTT.publish(altitude)) {
    Serial.println(F("Failed"));
  }

  if(! mqtt.ping()) {
    mqtt.disconnect();
  }
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  // Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}

//====================IN PROGRESS===================
void getLight(){
  lux = lightMeter.readLightLevel();

}

//===================WELL DONE=======================
void getBMP(){
  char result = bmp.startMeasurment();
  if(result!=0){
    delay(result);
    result = bmp.getTemperatureAndPressure(bmpTemp,pressure);
      if(result!=0){
        altitude = bmp.altitude(pressure,P0);
      }
      else {
        Serial.println("BMP Error, result == 0");
      }
  }
  else {
    Serial.println("BMP Error, result == 0");
  }
  delay(100);
}

void getDS18(){
  do {
    DS18B20.requestTemperatures(); 
    // Serial.println("request done");
    extTemp = DS18B20.getTempCByIndex(0);
    // Serial.println(extTemp);
  } while (extTemp == 85.0 || extTemp == (-127.0));

}

void serialPrint() {
  //bmp
  Serial.print("T = \t");Serial.print(bmpTemp,2); Serial.print(" degC\t");
  Serial.print("P = \t");Serial.print(pressure,2); Serial.print(" mBar\t");
  Serial.print("A = \t");Serial.print(altitude,2); Serial.println(" m");
  //ds18
  Serial.print("Temperature: ");
  Serial.println(extTemp);
  //dht
  Serial.print("internal: ");
  Serial.print(humid);
  Serial.print(" ");
  Serial.println(inTemp);
}

void getAccurateDHT(){
  humid = 0.0;
  inTemp = 0.0;
  for (int i = 0;i < 3; ++i) {
    delay(dht.getMinimumSamplingPeriod());
    humid += (dht.getHumidity())/3.0;
    inTemp += (dht.getTemperature())/3.0;
  }
  // if (humid == "nan" || inTemp == "nan") { 
  //   getAccurateDHT();
  // }
}
