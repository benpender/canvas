#include <WiFi.h>
#include <ESPAsyncE131.h>
#include <FastLED.h>
#include <string.h>

/* =======================
   USER CONFIG
   ======================= */

const char* WIFI_SSID = "WebPage";
const char* WIFI_PASS = "fivezerofourpage";

IPAddress localIP(10, 0, 0, 151);
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 255, 255, 0);

const uint16_t SACN_UNIVERSE = 1;

#define LED_PIN     14
#define MATRIX_W    8
#define MATRIX_H    8
#define NUM_LEDS    (MATRIX_W * MATRIX_H)
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define BRIGHTNESS  50
#define TARGET_FPS  60
#define FRAME_TIMEOUT_MS 2000

/* =======================
   GLOBALS
   ======================= */

CRGB leds[NUM_LEDS];
CRGB latestFrame[NUM_LEDS];
bool hasFrame = false;
ESPAsyncE131 e131(4);

unsigned long lastReportMs = 0;
unsigned long lastFrameMs = 0;
unsigned long lastRxMs = 0;
unsigned long lastRenderMs = 0;
uint32_t frameCount = 0;
uint32_t rxCount = 0;
int lastSeq = -1;
uint32_t seqDrops = 0;
uint32_t totalShowUs = 0;
uint32_t maxShowUs = 0;
uint32_t maxGapMs = 0;

/* =======================
   ART-NET CALLBACK
   ======================= */

void handleSacnPacket() {
  if (e131.isEmpty()) return;

  e131_packet_t packet;
  if (!e131.pull(&packet)) return;
  uint16_t universe = htons(packet.universe);
  if (universe != SACN_UNIVERSE) return;

  uint8_t sequence = packet.sequence_number;
  if (sequence != 0) {
    if (lastSeq >= 0) {
      uint8_t diff = (uint8_t)(sequence - (uint8_t)lastSeq);
      if (diff == 0 || diff > 128) return;
      if (diff > 1) seqDrops += (diff - 1);
    }
    lastSeq = sequence;
  }

  uint16_t slots = htons(packet.property_value_count);
  if (slots <= 1) return;
  uint16_t dmxLen = slots - 1;
  uint8_t *data = packet.property_values + 1;
  uint16_t ledCount = min(NUM_LEDS, dmxLen / 3);

  for (uint16_t i = 0; i < ledCount; i++) {
    latestFrame[i].r = data[i * 3 + 0];
    latestFrame[i].g = data[i * 3 + 1];
    latestFrame[i].b = data[i * 3 + 2];
  }
  hasFrame = true;
  lastRxMs = millis();
  rxCount += 1;
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
  Serial.print("WiFi IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("sACN universe: ");
  Serial.println(SACN_UNIVERSE);
  Serial.print("target_fps: ");
  Serial.println(TARGET_FPS);

  if (!e131.begin(E131_UNICAST, SACN_UNIVERSE, 1)) {
    Serial.println("sACN init failed");
  }
}

/* =======================
   LOOP
   ======================= */

void loop() {
  // Keep this HOT
  handleSacnPacket();

  unsigned long nowMs = millis();
  if (nowMs - lastRenderMs >= (1000 / TARGET_FPS)) {
    if (hasFrame && (nowMs - lastRxMs) <= FRAME_TIMEOUT_MS) {
      memcpy(leds, latestFrame, sizeof(leds));
    }

    uint32_t showStart = micros();
    FastLED.show();
    uint32_t showUs = micros() - showStart;

    frameCount += 1;
    totalShowUs += showUs;
    if (showUs > maxShowUs) {
      maxShowUs = showUs;
    }

    if (lastFrameMs > 0) {
      unsigned long gap = nowMs - lastFrameMs;
      if (gap > maxGapMs) {
        maxGapMs = gap;
      }
    }
    lastFrameMs = nowMs;
    lastRenderMs = nowMs;
  }

  if (nowMs - lastReportMs >= 1000) {
    uint32_t avgShowUs = frameCount ? (totalShowUs / frameCount) : 0;
    unsigned long frameAgeMs = hasFrame ? (nowMs - lastRxMs) : 0;
    Serial.printf("rates render_fps=%lu recv_fps=%lu seq_drops=%lu frame_age_ms=%lu avg_show_us=%lu max_show_us=%lu max_gap_ms=%lu\n",
                  (unsigned long)frameCount,
                  (unsigned long)rxCount,
                  (unsigned long)seqDrops,
                  (unsigned long)frameAgeMs,
                  (unsigned long)avgShowUs,
                  (unsigned long)maxShowUs,
                  (unsigned long)maxGapMs);
    frameCount = 0;
    rxCount = 0;
    seqDrops = 0;
    totalShowUs = 0;
    maxShowUs = 0;
    maxGapMs = 0;
    lastReportMs = nowMs;
  }
}
