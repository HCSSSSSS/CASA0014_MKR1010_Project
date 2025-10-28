// works with MKR1010

#include <SPI.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include "arduino_secrets.h"
#include <utility/wifi_drv.h>  // library to drive to RGB LED on the MKR1010

/*
**** please enter your sensitive data in the Secret tab/arduino_secrets.h
**** using format below
#define SECRET_SSID "ssid name"
#define SECRET_PASS "ssid password"
#define SECRET_MQTTUSER "user name - eg student"
#define SECRET_MQTTPASS "password";
 */
const char* ssid = SECRET_SSID;
const char* password = SECRET_PASS;
const char* mqtt_username = SECRET_MQTTUSER;
const char* mqtt_password = SECRET_MQTTPASS;
const char* mqtt_server = "mqtt.cetools.org";
const int mqtt_port = 1884;

// create wifi object and mqtt object
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// Make sure to update your lightid value below with the one you have been allocated
String lightId = "10";  // the topic id number or user number being used.

// Here we define the MQTT topic we will be publishing data to
String mqtt_topic = "student/CASA0014/luminaire/" + lightId;
String clientId = "";  // will set once i have mac address so that it is unique

// NeoPixel Configuration - we need to know this to know how to send messages
// to vespera
const int num_leds = 72;
const int payload_size = num_leds * 3;  // x3 for RGB

// Create the byte array to send in MQTT payload this stores all the colours
// in memory so that they can be accessed in for example the rainbow function
byte RGBpayload[payload_size];

//  FSR Sensor Configuration
#define SENSOR_PIN A0         // FSR signal input
const float EMA_ALPHA = 0.15;  // Exponential smoothing factor
float emaVal = 0;             // Smoothed sensor value

const int MIN_TRIG = 400;   // Minimum threshold
const int MAX_TRIG = 4095;  // Maximum threshold

const int ADC_MAX = 4095;
bool invertReading = true;  // if pressing make value smaller then true otherwise false

// Map 0..1 pressure to RGB: green -> yellow -> red
static inline void pressureToRGB(float t, uint8_t& r, uint8_t& g, uint8_t& b) {
  if (t < 0) t = 0;
  if (t > 1) t = 1;                    // clamp
  r = (uint8_t)(255.0f * t);           // more pressure -> more red
  g = (uint8_t)(255.0f * (1.0f - t));  // less green as pressure grows
  b = 0;
}

void setup() {
  Serial.begin(115200);
  //while (!Serial); // Wait for serial port to connect (useful for debugging)
  Serial.println("Vespera");


  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);

  Serial.print("This device is Vespera ");
  Serial.println(lightId);

  // Connect to WiFi
  startWifi();

  // Connect to MQTT broker
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setBufferSize(2000);
  mqttClient.setCallback(callback);

  analogReadResolution(12);         // MKR1010 ADC range: 0–4095
  emaVal = analogRead(SENSOR_PIN);  // Initialize smoothed value

  Serial.println("Set-up complete");
}

void loop() {
  // Reconnect if necessary
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }

  if (WiFi.status() != WL_CONNECTED) {
    startWifi();
  }
  // keep mqtt alive
  mqttClient.loop();

  //  Read FSR sensor and map to number of LEDs
  int raw = analogRead(SENSOR_PIN);        // Read analog value (0–4095)
  if (invertReading) raw = ADC_MAX - raw;  // reverse if needed

  // Apply exponential smoothing (EMA) emaVal = α × new value + (1 - α) × old value

  emaVal = EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * emaVal;

  // Map smoothed value to number of LEDs
  int rel = constrain((int)emaVal, MIN_TRIG, MAX_TRIG);
  int N = map(rel, MIN_TRIG, MAX_TRIG, 0, num_leds);
  N = constrain(N, 0, num_leds);

  //  limit how fast N can increase per frame
  static int lastN = 0;
  const int MAX_STEP_UP = 5;  // at most 5 LEDs per frame
  if (N > lastN + MAX_STEP_UP) N = lastN + MAX_STEP_UP;
  lastN = N;

  // --- compute color from pressure 
  float t = (float)(rel - MIN_TRIG) / (float)(MAX_TRIG - MIN_TRIG); 
  uint8_t r, g, b;
  pressureToRGB(t, r, g, b);  // get gradient color

  // --- fill payload: first N LEDs ON with (r,g,b), others OFF ---
  for (int i = 0; i < num_leds; i++) {
    if (i < N) {
      RGBpayload[i * 3 + 0] = r;  // R
      RGBpayload[i * 3 + 1] = g;  // G
      RGBpayload[i * 3 + 2] = b;  // B
    } else {
      RGBpayload[i * 3 + 0] = 0;
      RGBpayload[i * 3 + 1] = 0;
      RGBpayload[i * 3 + 2] = 0;
    }
  }


  // Publish the 216-byte payload to MQTT
  if (mqttClient.connected()) {
    mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);
  }

  delay(60);
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}