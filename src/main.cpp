#include <WiFi.h>
#include <ArtnetWifi.h>
#include <FastLED.h>

/* =======================
   USER CONFIG
   ======================= */

const char* WIFI_SSID = "WebPage";
const char* WIFI_PASS = "fivezerofourpage";

IPAddress localIP(10, 0, 0, 151);
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 255, 255, 0);

// IMPORTANT: this is WORKING for your sender
const uint16_t ARTNET_UNIVERSE = 1;

#define LED_PIN     14
#define MATRIX_W    8
#define MATRIX_H    8
#define NUM_LEDS    (MATRIX_W * MATRIX_H)
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define BRIGHTNESS  80

/* =======================
   GLOBALS
   ======================= */

CRGB leds[NUM_LEDS];
ArtnetWifi artnet;

/* =======================
   ART-NET CALLBACK
   ======================= */

void onArtNetFrame(
  uint16_t universe,
  uint16_t length,
  uint8_t sequence,
  uint8_t* data
) {
  if (universe != ARTNET_UNIVERSE) return;

  uint16_t ledCount = min(NUM_LEDS, length / 3);

  for (uint16_t i = 0; i < ledCount; i++) {
    leds[i].r = data[i * 3 + 0];
    leds[i].g = data[i * 3 + 1];
    leds[i].b = data[i * 3 + 2];
  }

  // YES — keep this here on ESP32
  FastLED.show();
}

/* =======================
   SETUP
   ======================= */

void setup() {
  Serial.begin(115200);
  unsigned long serialStart = millis();
  while (!Serial && (millis() - serialStart) < 5000) {
    delay(10);
  }

  Serial.println("\n===== ESP32-S3 BOOT =====");
  Serial.println("Starting...");
  delay(200);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear(true);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // 🔴 THIS MATTERS
  WiFi.config(localIP, gateway, subnet);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
  }

  artnet.begin();
  artnet.setArtDmxCallback(onArtNetFrame);
}

/* =======================
   LOOP
   ======================= */

void loop() {
  // Keep this HOT
  artnet.read();
}
