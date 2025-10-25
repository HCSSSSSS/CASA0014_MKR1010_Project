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
#define SENSOR_PIN A0              // FSR signal input
const bool INVERT_READING = true;  // If pressing makes the value smaller, set to true
const float EMA_ALPHA = 0.2;       // Exponential smoothing factor
float emaVal = 0;                  // Smoothed sensor value

const int MIN_TRIG = 300;   // Minimum threshold
const int MAX_TRIG = 4095;  // Maximum threshold


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
  int raw = analogRead(SENSOR_PIN);      // Read analog value (0–4095)
  if (INVERT_READING) raw = 4095 - raw;  // Reverse

  // Apply exponential smoothing (EMA)
  emaVal = EMA_ALPHA * raw + (1.0f - EMA_ALPHA) * emaVal;

  // Map smoothed value to number of LEDs
  int rel = constrain((int)emaVal, MIN_TRIG, 4095);
  int N = map(rel, MIN_TRIG, MAX_TRIG, 0, num_leds);
  N = constrain(N, 0, num_leds);

  //  limit how fast N can increase per frame
  static int lastN = 0;
  const int MAX_STEP_UP = 5;  // at most 5 LEDs per frame
  if (N > lastN + MAX_STEP_UP) N = lastN + MAX_STEP_UP;
  lastN = N;

  for (int i = 0; i < num_leds; i++) {
    if (i < N) {
      RGBpayload[i * 3 + 0] = 0;
      RGBpayload[i * 3 + 1] = 220;
      RGBpayload[i * 3 + 2] = 0;
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

  delay(60);  // Adjust refresh rate (16 updates per second)
}

/*
// Function to update the R, G, B values of a single LED pixel
// RGB can a value between 0-254, pixel is 0-71 for a 72 neopixel strip
void send_RGB_to_pixel(int r, int g, int b, int pixel) {
  // Check if the mqttClient is connected before publishing
  if (mqttClient.connected()) {
    // Update the byte array with the specified RGB color pattern
    RGBpayload[pixel * 3 + 0] = (byte)r;  // Red
    RGBpayload[pixel * 3 + 1] = (byte)g;  // Green
    RGBpayload[pixel * 3 + 2] = (byte)b;  // Blue

    // Publish the byte array
    mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);

    Serial.println("Published whole byte array after updating a single pixel.");
  } else {
    Serial.println("MQTT mqttClient not connected, cannot publish from *send_RGB_to_pixel*.");
  }
}

void send_all_off() {
  // Check if the mqttClient is connected before publishing
  if (mqttClient.connected()) {
    // Fill the byte array with the specified RGB color pattern
    for (int pixel = 0; pixel < num_leds; pixel++) {
      RGBpayload[pixel * 3 + 0] = (byte)0;  // Red
      RGBpayload[pixel * 3 + 1] = (byte)0;  // Green
      RGBpayload[pixel * 3 + 2] = (byte)0;  // Blue
    }
    // Publish the byte array
    mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);

    Serial.println("Published an all zero (off) byte array.");
  } else {
    Serial.println("MQTT mqttClient not connected, cannot publish from *send_all_off*.");
  }
}

void send_all_random() {
  // Check if the mqttClient is connected before publishing
  if (mqttClient.connected()) {
    // Fill the byte array with the specified RGB color pattern
    for (int pixel = 0; pixel < num_leds; pixel++) {
      RGBpayload[pixel * 3 + 0] = (byte)random(50, 256);  // Red - 256 is exclusive, so it goes up to 255
      RGBpayload[pixel * 3 + 1] = (byte)random(50, 256);  // Green
      RGBpayload[pixel * 3 + 2] = (byte)random(50, 256);  // Blue
    }
    // Publish the byte array
    mqttClient.publish(mqtt_topic.c_str(), RGBpayload, payload_size);

    Serial.println("Published an all random byte array.");
  } else {
    Serial.println("MQTT mqttClient not connected, cannot publish from *send_all_random*.");
  }
}
*/

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