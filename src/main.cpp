#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <SPIFFS.h>
#include <WiFiMulti.h>
#include <WiFiUdp.h>

#include "wifi_credentials.h"  // Store your wifi settings here

#define LED_BUILTIN 2

// Objects used
WiFiMulti multi;  // To store multiple wifi access points
WiFiClient espClient;
WiFiUDP ntpUDP;
PubSubClient client(espClient);                 // MQTT client
NTPClient timeClient(ntpUDP, "time.nist.gov");  // Get time from NTP servers

const char* ssid = WIFI_SSID;  // #define WIFI_SSID in wifi_credentials.h
const char* passwd = PASSWORD;

const char* SSID1 = "AndroidAPR";  // for mobile hotspot
const char* PASSWD1 = "androidapr";

const char* brokerHost = "broker.hivemq.com";  // broker address
uint16_t brokerPort = 1883;                    // broker port
const char* publishTopic =
    "esp32-iot-demo/freefall/A8F29CA4AE30";  // publish topic of freefall event

AsyncWebServer server(80);  // HTTP server
AsyncWebSocket ws("/ws");   // Websocket for realtime monitoring

Adafruit_MPU6050 mpu;  // MPU6050 object

void onMessage(char* topic, byte* payload, unsigned int length) {}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // At start-up of the device
  Serial.begin(115200);
  Serial.println("MPU6050 IoT Demo");

  WiFi.mode(WIFI_STA);

  multi.addAP(ssid, passwd);  // comment as necessory
  multi.addAP(SSID1, PASSWD1);
  multi.run();  // Try to connect to any accespoint configured

  if (!SPIFFS.begin(true)) {  // Setup SPI filesystem for webpages
    Serial.println("Formatting SPIFFS");
  }

  Serial.print("Connecting to WiFi");

  while (!WiFi.isConnected()) {  // Wati until wifi connection
    Serial.print('.');
    delay(500);
  }
  Serial.println();

  timeClient.begin();
  Serial.print("Updating time");

  while (!timeClient.update()) {  // Get and udpate time from NTP servers
    Serial.print('.');
    delay(500);
  };
  Serial.println();

  server.addHandler(&ws);

  // Serve webpages in SPI fat file system
  // Files in data folder can be transfered to SPIFFS
  // then they can be served as webpages at user requests
  server.serveStatic("/", SPIFFS, "/")
      .setDefaultFile("index.html")
      .setCacheControl("max-age=600");

  server.begin();

  if (!mpu.begin()) {  // Initialize MPU6050 connection
    Serial.println("Sensor init failed");
    while (1) yield();
  }
  Serial.println("Found a MPU6050 sensor");

  client.setServer(brokerHost, brokerPort);  // Setup MQTT broker
  client.setCallback(onMessage);

  Serial.print("IP: ");
  Serial.println(WiFi.localIP());  // Print IP of the device

  uint64_t chipid = ESP.getEfuseMac();  // The chip ID is essentially its MAC
                                        // address(length: 6 bytes).
  Serial.printf("ESP32 Chip ID = %04X",
                (uint16_t)(chipid >> 32));      // print High 2 bytes
  Serial.printf("%08X\r\n", (uint32_t)chipid);  // print Low 4bytes.
  digitalWrite(LED_BUILTIN, LOW);               // Device initilized!
}

uint64_t startTime = 0;
uint16_t period = 100;
uint16_t lastFreefall = 0;
float sumAcc = 0;
float accThreshold = 1.5;

uint64_t lastTriedAt = 0;

void loop() {
  ws.cleanupClients();
  client.loop();  // Must include to handle MQTT traffic

  if (millis() - startTime > period) {  // Run in 100 ms loops
    startTime = millis();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);  // get acceleration components
    ws.printfAll("%.2f %.2f %.2f", a.acceleration.x, a.acceleration.y,
                 a.acceleration.z);  // send though websocket
    sumAcc = abs(a.acceleration.x) + abs(a.acceleration.y) +
             abs(a.acceleration.z);  // Sum of all vector components

    digitalWrite(LED_BUILTIN, sumAcc < accThreshold);

    // If sum is less than freefall threshold and after 3 seconds from last
    // freefall
    if ((sumAcc < accThreshold) && ((millis() - lastFreefall) > 3000)) {
      lastFreefall = millis();
      uint32_t timestamp = timeClient.getEpochTime();  // Get UNIX timestamp
      char message[200];                               // message buffer
      sprintf(message, "%s %d", WiFi.macAddress().c_str(),
              timestamp);                     // fabricate the message
      client.publish(publishTopic, message);  // publish to MQTT broker !
    }
  }

  // Reconnect to MQTT broker at connection failure
  if (!client.connected() && ((millis() - lastTriedAt) > 5000)) {
    digitalWrite(LED_BUILTIN, HIGH);
    lastTriedAt = millis();
    Serial.println("Attempting MQTT connection...");
    if (!client.connect("clientID_A8F29CA4AE30")) {
      Serial.print("failed, rc=");
      Serial.print(client.state());
    } else {
      Serial.println("MQTT Connected !");
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}
