#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <SPIFFS.h>
#include <WiFiMulti.h>
#include <WiFiUdp.h>

#include "wifi_credentials.h"

#define LED_BUILTIN 2

WiFiMulti multi;
WiFiClient espClient;
WiFiUDP ntpUDP;
PubSubClient client(espClient);
NTPClient timeClient(ntpUDP, "time.nist.gov");

const char* ssid = WIFI_SSID;
const char* passwd = PASSWORD;

const char* SSID1 = "AndroidAPR";
const char* PASSWD1 = "androidapr";

const char* brokerHost = "broker.hivemq.com";
uint16_t brokerPort = 1883;
const char* publishTopic = "esp32-iot-demo/freefall/A8F29CA4AE30";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

Adafruit_MPU6050 mpu;

void onMessage(char* topic, byte* payload, unsigned int length) {}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);
  Serial.println("MPU6050 IoT Demo");

  WiFi.mode(WIFI_STA);

  multi.addAP(ssid, passwd);
  multi.addAP(SSID1, PASSWD1);
  multi.run();

  if (!SPIFFS.begin(true)) {
    Serial.println("Formatting SPIFFS");
  }

  Serial.print("Connecting to WiFi");

  while (!WiFi.isConnected()) {
    Serial.print('.');
    delay(500);
  }
  Serial.println();

  timeClient.begin();
  Serial.print("Updating time");

  while (!timeClient.update()) {
    Serial.print('.');
    delay(500);
  };
  Serial.println();

  server.addHandler(&ws);

  server.serveStatic("/", SPIFFS, "/")
      .setDefaultFile("index.html")
      .setCacheControl("max-age=600");

  server.begin();

  if (!mpu.begin()) {
    Serial.println("Sensor init failed");
    while (1) yield();
  }
  Serial.println("Found a MPU6050 sensor");

  client.setServer(brokerHost, brokerPort);
  client.setCallback(onMessage);

  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  uint64_t chipid = ESP.getEfuseMac();  // The chip ID is essentially its MAC
                                        // address(length: 6 bytes).
  Serial.printf("ESP32 Chip ID = %04X",
                (uint16_t)(chipid >> 32));      // print High 2 bytes
  Serial.printf("%08X\r\n", (uint32_t)chipid);  // print Low 4bytes.
  digitalWrite(LED_BUILTIN, LOW);
}

uint64_t startTime = 0;
uint16_t period = 100;
uint16_t lastFreefall = 0;
float sumAcc = 0;
float accThreshold = 1.5;

uint64_t lastTriedAt = 0;

void loop() {
  ws.cleanupClients();
  client.loop();

  if (millis() - startTime > period) {
    startTime = millis();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    ws.printfAll("%.2f %.2f %.2f", a.acceleration.x, a.acceleration.y,
                 a.acceleration.z);
    sumAcc =
        abs(a.acceleration.x) + abs(a.acceleration.y) + abs(a.acceleration.z);

    digitalWrite(LED_BUILTIN, sumAcc < accThreshold);

    if ((sumAcc < accThreshold) && ((millis() - lastFreefall) > 3000)) {
      lastFreefall = millis();
      uint32_t timestamp = timeClient.getEpochTime();
      char message[200];
      sprintf(message, "%s %d", WiFi.macAddress().c_str(), timestamp);
      client.publish(publishTopic, message);
    }
  }

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
