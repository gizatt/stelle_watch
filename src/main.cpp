#include <Arduino_GFX_Library.h>
#include <math.h>

// Display setup (same as yours)
Arduino_DataBus *bus = new Arduino_ESP32SPI(
  GFX_NOT_DEFINED /* DC */, 42 /* CS */,
  2 /* SCK */, 1 /* MOSI */, GFX_NOT_DEFINED /* MISO */);

Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
  40 /* DE */, 39 /* VSYNC */, 38 /* HSYNC */, 41 /* PCLK */,
  46 /* R0 */, 3 /* R1 */, 8 /* R2 */, 18 /* R3 */, 17 /* R4 */,
  14 /* G0 */, 13 /* G1 */, 12 /* G2 */, 11 /* G3 */, 10 /* G4 */, 9 /* G5 */,
  5 /* B0 */, 45 /* B1 */, 48 /* B2 */, 47 /* B3 */, 21 /* B4 */,
  1 /* hsync_polarity */, 10, 8, 10,
  1 /* vsync_polarity */, 10, 8, 10);

Arduino_RGB_Display *panel = new Arduino_RGB_Display(
  480, 480, rgbpanel, 0, false,
  bus, GFX_NOT_DEFINED, st7701_type6_init_operations, sizeof(st7701_type6_init_operations));
Arduino_Canvas *gfx = new Arduino_Canvas(480, 480, panel);


// === Clock Config ===
const float MINUTES_PER_SECOND = 10.0f;
const int CENTER_X = 240;
const int CENTER_Y = 240;
const int RADIUS = 200;

// === Color Defines ===
#define CLOCK_FACE_COLOR WHITE
#define HOUR_HAND_COLOR RGB565(200, 200, 200)
#define MINUTE_HAND_COLOR RGB565(180, 180, 180)
#define SECOND_HAND_COLOR RGB565(160, 160, 160)

// === Forward declarations ===
void drawHand(float angleDeg, float length, uint16_t color, int thickness);

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

// === Helper to draw a clock hand ===
void drawHand(float angleDeg, float length, uint16_t color, int thickness)
{
  float angleRad = (angleDeg - 90) * DEG_TO_RAD; // 0 deg = 12 o'clock
  int x = CENTER_X + cos(angleRad) * length;
  int y = CENTER_Y + sin(angleRad) * length;
  for (int dx = -thickness / 2; dx <= thickness / 2; dx++) {
    for (int dy = -thickness / 2; dy <= thickness / 2; dy++) {
      gfx->drawLine(CENTER_X + dx, CENTER_Y + dy, x, y, color);
    }
  }
}

// === Main loop ===
void loop()
{
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  if (now - lastUpdate >= 100) {
    lastUpdate = now;

    float elapsedSeconds = now / 1000.0f;
    float totalMinutes = elapsedSeconds * MINUTES_PER_SECOND;
    int hours = ((int)(totalMinutes / 60)) % 12;
    int minutes = ((int)totalMinutes) % 60;
    int seconds = ((int)(totalMinutes * 60)) % 60;

    gfx->fillScreen(BLACK);

    // Draw outer clock ring and tick marks
    gfx->drawCircle(CENTER_X, CENTER_Y, RADIUS, CLOCK_FACE_COLOR);
    for (int i = 0; i < 12; i++) {
      float angle = i * 30 * DEG_TO_RAD;
      int x1 = CENTER_X + cos(angle) * (RADIUS - 10);
      int y1 = CENTER_Y + sin(angle) * (RADIUS - 10);
      int x2 = CENTER_X + cos(angle) * (RADIUS - 20);
      int y2 = CENTER_Y + sin(angle) * (RADIUS - 20);
      gfx->drawLine(x1, y1, x2, y2, CLOCK_FACE_COLOR);
    }

    // Calculate hand angles
    float hourAngle = ((hours % 12) + minutes / 60.0f) * 30.0f;
    float minuteAngle = minutes * 6.0f;
    float secondAngle = seconds * 6.0f;

    // Draw hands
    drawHand(hourAngle, RADIUS * 0.5, HOUR_HAND_COLOR, 6);
    drawHand(minuteAngle, RADIUS * 0.75, MINUTE_HAND_COLOR, 4);
    drawHand(secondAngle, RADIUS * 0.9, SECOND_HAND_COLOR, 2);

    gfx->flush();
  }
}
