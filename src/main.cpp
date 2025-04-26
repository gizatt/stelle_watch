#include "ParticleSim.h"
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
    48 /* B2 */, 47 /* B3 */, 21 /* B4 */, 1 /* hsync_polarity */,
    20 /* hsync front porch */, 10 /* hsync pulse width */,
    10 /*hsync back porch*/, 1 /* vsync_polarity */, 10 /* vsync front porch */,
    10 /* vsync pulse width */, 10 /* vsync back porch*/, 0 /* pckl active neg */,
    GFX_NOT_DEFINED /* prefer speed */, false /* use big endian */);


#define BACKLIGHT_PIN 6

#define NUMARGS(...)  (sizeof((int[]){__VA_ARGS__})/sizeof(int))

#define ESP_PANEL_LCD_CMD_WITH_8BIT_PARAM(cmd, ...) \
  BEGIN_WRITE,                                         \
  WRITE_COMMAND_8, cmd,                                \
  WRITE_BYTES, NUMARGS(__VA_ARGS__), __VA_ARGS__,                \
  END_WRITE

// https://focuslcds.com/wp-content/uploads/Drivers/ST7701S.pdf?srsltid=AfmBOopYK3aCfyL_tB86MdP0n324VHZnOGzSyoDPjkvCC9OespEM7N0c
static const uint8_t st7701_my_init_operations[] = {
  BEGIN_WRITE,
  WRITE_COMMAND_8, 0xFF,
  WRITE_BYTES, 5, 0x77, 0x01, 0x00, 0x00, 0x13,
  WRITE_C8_D8, 0xEF, 0x08,

  WRITE_COMMAND_8, 0xFF,
  WRITE_BYTES, 5, 0x77, 0x01, 0x00, 0x00, 0x10,

  WRITE_C8_D16, 0xC0, 0x3B, 0x00,

  WRITE_C8_D16, 0xC1, 0x10, 0x0C,

  WRITE_C8_D16, 0xC2, 0x07, 0x0A,

  WRITE_C8_D8, 0xC7, 0x00,

  WRITE_C8_D8, 0xCC, 0x10,

  WRITE_COMMAND_8, 0xB0,
  WRITE_BYTES, 16,
  0x05, 0x12, 0x98, 0x0E,
  0x0F, 0x07, 0x07, 0x09,
  0x09, 0x23, 0x05, 0x52,
  0x0F, 0x67, 0x2C, 0x11,

  WRITE_COMMAND_8, 0xB1,
  WRITE_BYTES, 16,
  0x0B, 0x11, 0x97, 0x0C,
  0x12, 0x06, 0x06, 0x08,
  0x08, 0x22, 0x03, 0x51,
  0x11, 0x66, 0x2B, 0x0F,

  WRITE_COMMAND_8, 0xFF,
  WRITE_BYTES, 5, 0x77, 0x01, 0x00, 0x00, 0x11,

  WRITE_C8_D8, 0xB0, 0x5D,
  WRITE_C8_D8, 0xB1, 0x2D,
  WRITE_C8_D8, 0xB2, 0x81,
  WRITE_C8_D8, 0xB3, 0x80,

  WRITE_C8_D8, 0xB5, 0x4E,

  WRITE_C8_D8, 0xB7, 0x85,
  WRITE_C8_D8, 0xB8, 0x20,

  WRITE_C8_D8, 0xC1, 0x78,
  WRITE_C8_D8, 0xC2, 0x78,

  WRITE_C8_D8, 0xD0, 0x88,

  WRITE_COMMAND_8, 0xE0,
  WRITE_BYTES, 3, 0x00, 0x00, 0x02,

  WRITE_COMMAND_8, 0xE1,
  WRITE_BYTES, 11,
  0x06, 0x30, 0x08, 0x30,
  0x05, 0x30, 0x07, 0x30,
  0x00, 0x33, 0x33,

  WRITE_COMMAND_8, 0xE2,
  WRITE_BYTES, 12,
  0x11, 0x11, 0x33, 0x33,
  0xF4, 0x00, 0x00, 0x00,
  0xF4, 0x00, 0x00, 0x00,

  WRITE_COMMAND_8, 0xE3,
  WRITE_BYTES, 4, 0x00, 0x00, 0x11, 0x11,

  WRITE_C8_D16, 0xE4, 0x44, 0x44,

  WRITE_COMMAND_8, 0xE5,
  WRITE_BYTES, 16,
  0x0D, 0xF5, 0x30, 0xF0,
  0x0F, 0xF7, 0x30, 0xF0,
  0x09, 0xF1, 0x30, 0xF0,
  0x0B, 0xF3, 0x30, 0xF0,

  WRITE_COMMAND_8, 0xE6,
  WRITE_BYTES, 4, 0x00, 0x00, 0x11, 0x11,

  WRITE_C8_D16, 0xE7, 0x44, 0x44,

  WRITE_COMMAND_8, 0xE8,
  WRITE_BYTES, 16,
  0x0C, 0xF4, 0x30, 0xF0,
  0x0E, 0xF6, 0x30, 0xF0,
  0x08, 0xF0, 0x30, 0xF0,
  0x0A, 0xF2, 0x30, 0xF0,

  WRITE_C8_D16, 0xE9, 0x36, 0x01,

  WRITE_COMMAND_8, 0xEB,
  WRITE_BYTES, 7,
  0x00, 0x01, 0xE4, 0xE4,
  0x44, 0x88, 0x40,

  WRITE_COMMAND_8, 0xED,
  WRITE_BYTES, 16,
  0xFF, 0x10, 0xAF, 0x76,
  0x54, 0x2B, 0xCF, 0xFF,
  0xFF, 0xFC, 0xB2, 0x45,
  0x67, 0xFA, 0x01, 0xFF,

  WRITE_COMMAND_8, 0xEF,
  WRITE_BYTES, 6,
  0x08, 0x08, 0x08, 0x45,
  0x3F, 0x54,

  WRITE_COMMAND_8, 0xFF,
  WRITE_BYTES, 5, 0x77, 0x01, 0x00, 0x00, 0x00,

  WRITE_COMMAND_8, 0x11,
  END_WRITE,

  DELAY, 120, // ms

  BEGIN_WRITE,
  WRITE_C8_D8, 0x3A, 0x66,

  WRITE_C8_D8, 0x36, 0x00,

  WRITE_C8_D8, 0x35, 0x00,

  WRITE_COMMAND_8, 0x29, // Display On
  END_WRITE};


// type 3, 4, 7: white screen
// type 6: mostly right. went weirdly green for a while. flashed with a different
// type, power cycled, reflashed type 6, now it's white background instead of black???
// type 8 & 9: black background, random white and green lines being drawn. it's allllmost right. wrong clock?
// 
Arduino_RGB_Display *panel = new Arduino_RGB_Display(
    480, 480, rgbpanel, 0, false, bus, GFX_NOT_DEFINED,
    st7701_my_init_operations, sizeof(st7701_my_init_operations));
Arduino_Canvas *gfx = new Arduino_Canvas(480, 480, panel);

ParticleSim *sim = new ParticleSim();
Mat3xNf last_positions;

double start_t = 0.0;
inline double get_now() { return ((double)millis()) / 1000.0f; }

void setup() {
  Serial.begin(115200);

  pinMode(BACKLIGHT_PIN, OUTPUT);
  digitalWrite(BACKLIGHT_PIN, 0);

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

  sim->initialize_all_particles();
  last_positions.resize(3, N_PARTICLES);
  last_positions.setZero();
  start_t = get_now();
}

// === Main loop ===
int n_frames = 0;
double last_draw = 0.0;

inline void draw_pos(const Vec3f &p, uint16_t color) {
  int u = (int)(p(0) * 100 + 240);
  int v = (int)(p(1) * 100 + 240);
  if (u < 1 || u >= 479 || v < 0 || v >= 479) {
    return;
  }
  gfx->writePixel(u, v, color);
  // Draw neighboring pixels as well for a little "star" effect
  // gfx->drawPixel(u+1, v, color);
  // gfx->drawPixel(u-1, v, color);
  // gfx->drawPixel(u, v+1, color);
  // gfx->drawPixel(u, v-1, color);
}

void loop() {
  double t = get_now();

  sim->dynamics_update(t);

  if (t - last_draw >= 1. / 30.0) {
    last_draw = t;
    n_frames++;
    double fps = n_frames / (t - start_t);
    Serial.printf("FPS: %.2f\n", fps);

    // We instead undraw previous pixels to save screen clearing time.
    // gfx->fillScreen(BLACK);

    const auto &positions = sim->get_positions();
    const auto &colors = sim->get_colors();
    for (int i = 0; i < N_PARTICLES; ++i) {
      // Undraw last position.
      const auto &last_pos = last_positions.col(i);
      draw_pos(last_positions.col(i), BLACK);

      // Draw new position.
      const auto &pos = positions.col(i);
      const auto &color = colors.col(i);
      draw_pos(pos, gfx->color565(color(0), color(1), color(2)));
      last_positions.col(i) = pos;
    }

    gfx->flush();
  }
}
