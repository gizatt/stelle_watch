#include "DisplayMagic.h"
#include "ParticleSim.h"
#include <Arduino_GFX_Library.h>
#include <math.h>


// type 3, 4, 7: white screen
// type 6: mostly right. went weirdly green for a while. flashed with a
// different type, power cycled, reflashed type 6, now it's white background
// instead of black??? type 8 & 9: black background, random white and green
// lines being drawn. it's allllmost right. wrong clock?
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
