#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <SensorQMI8658.hpp>
#include <WebSocketsServer.h>
#include <math.h>

const char* WIFI_SSID = "WebPage";
const char* WIFI_PASS = "fivezerofourpage";

IPAddress localIP(10, 0, 0, 151);
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 255, 255, 0);

WebSocketsServer webSocket(81);

// Define the I2C pins for the Waveshare ESP32-S3-Matrix
#define IMU_SDA 11
#define IMU_SCL 12

// Define the sensor object
SensorQMI8658 qmi;

// Struct to hold sensor data
IMUdata acc;
IMUdata gyr;
float yawDeg = 0.0f;
uint8_t wsClientCount = 0;
uint32_t lastLogMs = 0;
uint32_t lastWifiCheckMs = 0;

void onWsEvent(uint8_t /*num*/, WStype_t type, uint8_t* /*payload*/, size_t /*length*/) {
  if (type == WStype_CONNECTED) {
    wsClientCount = min<uint8_t>(255, wsClientCount + 1);
    Serial.printf("WebSocket client connected. clients=%u\n", wsClientCount);
  } else if (type == WStype_DISCONNECTED) {
    wsClientCount = wsClientCount > 0 ? wsClientCount - 1 : 0;
    Serial.printf("WebSocket client disconnected. clients=%u\n", wsClientCount);
  }
}

float wrap180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // Wait for Serial to be ready (useful for USB-C debugging)
  // You might want to remove this if you want it to run without a PC connected immediately
  delay(2000); 
  Serial.println("Starting ESP32-S3-Matrix IMU Demo...");

  // Initialize the QMI8658 sensor
  // Usage: .begin(WireInstance, Address, SDA_Pin, SCL_Pin)
  // Address is usually 0x6B for this board
  if (!qmi.begin(Wire, QMI8658_ADDRESS_HIGH, IMU_SDA, IMU_SCL)) {
    Serial.println("Failed to find QMI8658 - Check your wiring or pin definitions!");
    while (1) { delay(1000); }
  }

  Serial.println("QMI8658 Sensor found!");
  
  // Optional: Configure sensor (Rates, Scales, etc.)
  qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_500Hz, SensorQMI8658::LPF_MODE_0);
  qmi.configGyroscope(SensorQMI8658::GYRO_RANGE_512DPS, SensorQMI8658::GYRO_ODR_500Hz, SensorQMI8658::LPF_MODE_0);
  
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.config(localIP, gateway, subnet);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }
  Serial.print("WiFi connected. IP=");
  Serial.println(WiFi.localIP());

  webSocket.begin();
  webSocket.enableHeartbeat(2000, 6000, 2);
  webSocket.onEvent(onWsEvent);
  Serial.println("Sensor Configured. Streaming IMU over WebSocket...");
}

void loop() {
  static constexpr uint32_t kSampleIntervalUs = 16667; // ~60 Hz
  static uint32_t lastSampleUs = 0;

  webSocket.loop();

  const uint32_t nowMs = millis();
  if (nowMs - lastWifiCheckMs >= 1000) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi lost. Reconnecting...");
      WiFi.disconnect(false);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
    lastWifiCheckMs = nowMs;
  }

  const uint32_t nowUs = micros();
  if (lastSampleUs == 0) {
    lastSampleUs = nowUs;
    return;
  }
  if (nowUs - lastSampleUs < kSampleIntervalUs) {
    return;
  }
  const float dt = (nowUs - lastSampleUs) / 1000000.0f;
  lastSampleUs = nowUs;

  // Check if data is available from the sensor
  if (qmi.getDataReady()) {
    // Read the data into our structs
    if (qmi.getAccelerometer(acc.x, acc.y, acc.z) && 
        qmi.getGyroscope(gyr.x, gyr.y, gyr.z)) {
      const float ax = acc.x / 1000.0f;
      const float ay = acc.y / 1000.0f;
      const float az = acc.z / 1000.0f;

      const float roll = atan2f(ay, az) * 180.0f / PI;
      const float pitch = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * 180.0f / PI;

      yawDeg = wrap180(yawDeg + gyr.z * dt);

      if (wsClientCount > 0) {
        char payload[96];
        const int len = snprintf(payload, sizeof(payload),
                                 "{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}",
                                 roll, pitch, yawDeg);
        if (len > 0) {
          webSocket.broadcastTXT(payload, len);
        }
      }

      const uint32_t nowMs = millis();
      if (nowMs - lastLogMs >= 1000) {
        Serial.printf("IMU roll=%.2f pitch=%.2f yaw=%.2f acc=[%.1f %.1f %.1f] gyr=[%.2f %.2f %.2f]\n",
                      roll, pitch, yawDeg, acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z);
        lastLogMs = nowMs;
      }
    }
  }
}
