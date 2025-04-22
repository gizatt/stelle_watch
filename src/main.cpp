#include <Arduino_GFX_Library.h>
#include <math.h>

// Display setup (same as yours)
Arduino_DataBus *bus =
    new Arduino_ESP32SPI(GFX_NOT_DEFINED /* DC */, 42 /* CS */, 2 /* SCK */,
                         1 /* MOSI */, GFX_NOT_DEFINED /* MISO */);

Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    40 /* DE */, 39 /* VSYNC */, 38 /* HSYNC */, 41 /* PCLK */, 46 /* R0 */,
    3 /* R1 */, 8 /* R2 */, 18 /* R3 */, 17 /* R4 */, 14 /* G0 */, 13 /* G1 */,
    12 /* G2 */, 11 /* G3 */, 10 /* G4 */, 9 /* G5 */, 5 /* B0 */, 45 /* B1 */,
    48 /* B2 */, 47 /* B3 */, 21 /* B4 */, 1 /* hsync_polarity */, 10, 8, 10,
    1 /* vsync_polarity */, 10, 8, 10);

Arduino_RGB_Display *panel = new Arduino_RGB_Display(
    480, 480, rgbpanel, 0, false, bus, GFX_NOT_DEFINED,
    st7701_type6_init_operations, sizeof(st7701_type6_init_operations));
Arduino_Canvas *gfx = new Arduino_Canvas(480, 480, panel);

// === Clock Config ===
const float SECONDS_PER_SECOND = 60.0 * 10.0f;
const int CENTER_X = 240;
const int CENTER_Y = 240;
const int RADIUS = 200;

// === Color Defines and Helpers ===
#define CLOCK_FACE_COLOR RGB565(100, 100, 150)
#define HOUR_HAND_COLOR RGB565(200, 200, 200)
#define MINUTE_HAND_COLOR RGB565(180, 180, 180)
#define SECOND_HAND_COLOR RGB565(160, 160, 160)
#define SPARKLE_BASE_COLOR RGB565(200, 220, 240) // soft icy blue

uint16_t scaleColor(uint16_t color, uint8_t brightness) {
  uint8_t r = ((color >> 11) & 0x1F) * 255 / 31;
  uint8_t g = ((color >> 5) & 0x3F) * 255 / 63;
  uint8_t b = (color & 0x1F) * 255 / 31;

  r = (r * brightness) / 255;
  g = (g * brightness) / 255;
  b = (b * brightness) / 255;

  return RGB565(r, g, b);
}

static inline float randomFloat(float min, float max) {
  return min + (max - min) * ((float)random()) / ((float)RAND_MAX);
}

// === Helper to draw a clock hand ===
void drawHand(float angleDeg, float length, uint16_t color, int thickness) {
  float angleRad = (angleDeg - 90) * DEG_TO_RAD; // 0 deg = 12 o'clock
  int x = CENTER_X + cos(angleRad) * length;
  int y = CENTER_Y + sin(angleRad) * length;
  for (int dx = -thickness / 2; dx <= thickness / 2; dx++) {
    for (int dy = -thickness / 2; dy <= thickness / 2; dy++) {
      gfx->drawLine(CENTER_X + dx, CENTER_Y + dy, x, y, color);
    }
  }
}

// === Sparkle config ===
#define MAX_SPARKLES 1000
#define SPARKLE_LIFETIME 2000 // in ms
#define SPARKLE_RATE_PER_SECOND 250.0f

class Sparkle {
private:
  bool m_active = false;
  int m_x = 0, m_y = 0;
  unsigned long m_birth = 0;
  unsigned long m_lifetime = 1000;
  uint16_t m_color = SPARKLE_BASE_COLOR;

public:
  bool active() { return m_active; }

  void activate(unsigned long now) {
    m_x = random(10, gfx->width() - 10);
    m_y = random(10, gfx->height() - 10);
    m_birth = now;
    m_lifetime = SPARKLE_LIFETIME + SPARKLE_LIFETIME * randomFloat(0.5, 1.5);
    m_active = true;

    // Apply slight variation in base color
    uint8_t r = 180 + random(-20, 20);
    uint8_t g = 200 + random(-20, 20);
    uint8_t b = 220 + random(-20, 20);
    m_color = RGB565(constrain(r, 0, 255), constrain(g, 0, 255),
                     constrain(b, 0, 255));
  }

  void update(unsigned long now) {
    if (m_active && now - m_birth > m_lifetime) {
      m_active = false;
    }
  }

  void draw(Arduino_Canvas *gfx, unsigned long now) {
    if (!m_active)
      return;

    float t = (now - m_birth) / (float)m_lifetime;
    float alpha = 1.0f - fabs(2 * t - 1); // fades in and out
    uint8_t brightness = (uint8_t)(alpha * 255);
    uint16_t color = scaleColor(m_color, brightness);

    gfx->drawPixel(m_x, m_y, color);
    gfx->drawPixel(m_x + 1, m_y, color);
    gfx->drawPixel(m_x - 1, m_y, color);
    gfx->drawPixel(m_x, m_y + 1, color);
    gfx->drawPixel(m_x, m_y - 1, color);
  }
};

Sparkle sparkles[MAX_SPARKLES];

void setup() {
  Serial.begin(115200);

  if (!psramFound()) {
    while (1) {
      Serial.println("No PSRAM found! Can't use double buffering.");
      delay(100);
    }
  }

  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(BLACK);
}

// === Main loop ===
void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  double dt = ((double)now - lastUpdate) / 1000.0f; // in seconds

  if (dt >= 0.03) {
    lastUpdate = now;

    float elapsedSeconds = SECONDS_PER_SECOND * now / 1000.0f;
    int hours = (int)(elapsedSeconds / 60 / 60) % 12;
    int minutes = ((int)elapsedSeconds / 60) % 60;
    int seconds = (int)elapsedSeconds;

    gfx->fillScreen(BLACK);

    if (randomFloat(0.0, 1.0) < SPARKLE_RATE_PER_SECOND * dt) {
      for (int i = 0; i < MAX_SPARKLES; i++) {
        if (!sparkles[i].active()) {
          sparkles[i].activate(now);
          break;
        }
      }
    }
    // Update and draw sparkles
    for (int i = 0; i < MAX_SPARKLES; i++) {
      sparkles[i].update(now);
      sparkles[i].draw(gfx, now);
    }

    // Draw outer clock ring and tick marks
    // gfx->drawCircle(CENTER_X, CENTER_Y, RADIUS, CLOCK_FACE_COLOR);
    const float radians_per_pixel = 1. / RADIUS;
    int thickness = 3; // Desired thickness for the tick marks
    for (int i = 0; i < 12; i++) {
      float angle = i * 30 * DEG_TO_RAD;
      int x1 = CENTER_X + cos(angle) * (RADIUS - 10);
      int y1 = CENTER_Y + sin(angle) * (RADIUS - 10);
      int x2 = CENTER_X + cos(angle) * (RADIUS - 20);
      int y2 = CENTER_Y + sin(angle) * (RADIUS - 20);
      for (int dx = -thickness / 2; dx <= thickness / 2; dx++) {
        for (int dy = -thickness / 2; dy <= thickness / 2; dy++) {
          gfx->drawLine(x1 + dx, y1 + dy, x2 + dx, y2 + dy, CLOCK_FACE_COLOR);
        }
      }
    }

    // Calculate hand angles
    float hourAngle = ((hours % 12) + minutes / 60.0f) * 30.0f;
    float minuteAngle = minutes * 6.0f;
    float secondAngle = seconds * 6.0f;

    // Draw hands
    drawHand(hourAngle, RADIUS * 0.5, HOUR_HAND_COLOR, 6);
    drawHand(minuteAngle, RADIUS * 0.75, MINUTE_HAND_COLOR, 4);
    // Skip the second hand if we're going so fast it'll just alias.
    if (SECONDS_PER_SECOND < 30.) {
      drawHand(secondAngle, RADIUS * 0.9, SECOND_HAND_COLOR, 2);
    }

    gfx->flush();
  }
}
