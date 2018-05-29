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
#include "wifi.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

float inTemp,humid,extTemp;
double bmpTemp,pressure,altitude,lux;

const short ds18pin = 14, bmpsda = 5, bmpscl = 4, dhtpin = 14;

BMP280 bmp;

DHTesp dht;
OneWire oneWire(ds18pin);
DallasTemperature DS18B20(&oneWire);

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, "192.168.100.100", 1883);
Adafruit_MQTT_Publish extTempMQTT = Adafruit_MQTT_Publish(&mqtt, "externalTemp");
Adafruit_MQTT_Publish inTempMQTT = Adafruit_MQTT_Publish(&mqtt, "inTemp");
Adafruit_MQTT_Publish humidMQTT = Adafruit_MQTT_Publish(&mqtt, "humid");
Adafruit_MQTT_Publish bmpTempMQTT = Adafruit_MQTT_Publish(&mqtt, "bmpTemp");
Adafruit_MQTT_Publish pressureMQTT = Adafruit_MQTT_Publish(&mqtt, "pressure");
Adafruit_MQTT_Publish lightMQTT = Adafruit_MQTT_Publish(&mqtt, "light");

BH1750 lightMeter(0x23);

void setup(){
  Serial.begin(115200);
    Serial.setTimeout(2000);

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
      Serial.println("BMP init failed!\n Reset in 10 seconds");
      delay(10000);
      ESP.reset();
  }
  else {
    Serial.println("BMP init success!");  
    bmp.setOversampling(4);
  }
  
  // Wire.begin(lightSCL, lightSDA);
  if (!lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))
    Serial.println("lightMeter error!");

  // short i = 0;

  // while (i < 1) {  
    // ++i;
  // }
  // Serial.println("lets sleep for 30e6 us or 30 seconds");
  // ESP.deepSleep(30e6); 
}

void loop(){
    // getDS18();
    getBMP();
    getLight();
    MQTT_loop();
    getAccurateDHT();
    serialPrint();
}


//====================IN PROGRESS===================

//===================WELL DONE=======================
void getBMP(){
  char result = bmp.startMeasurment();
  if(result!=0){
    delay(result);
    bmp.getTemperatureAndPressure(bmpTemp,pressure);
  }
}

void getDS18(){
  do {
    DS18B20.requestTemperatures(); 
    extTemp = DS18B20.getTempCByIndex(0);
  } while (extTemp == 85.0 || extTemp == (-127.0));
}

void getLight(){
  lux = lightMeter.readLightLevel();
}

void serialPrint() {
  Serial.println("====");
  Serial.println("BMP280 Temperature: " + String(bmpTemp) + "degC");
  Serial.println("Pressure: " + String(pressure) + "mBar");
  Serial.println("DS18B20 Temperature: " + String(extTemp) + "degC");
  Serial.println("DHT11 Temperature: " + String(inTemp) + "degC");
  Serial.println("Humidity: " + String(humid) + "%");
  Serial.println("Light:" + String(lux)+"lux");
}

void getAccurateDHT(){
  // do {
    humid = (dht.getHumidity());
    inTemp = (dht.getTemperature());
  // } while ((humid == NAN)||(inTemp == NAN));
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
  if (! lightMQTT.publish(lux)) {
    Serial.println(F("Failed"));
  }

  if(! mqtt.ping()) {
    mqtt.disconnect();
  }
}

void MQTT_connect() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
    while (1);
  }
  }
  Serial.println("MQTT Connected!");
}
