#include "DisplayMagic.h"
#include "ParticleSim.h"
#include <Arduino_GFX_Library.h>
#include <math.h>
#include "nebula_bitmap.h"

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

double start_t = 0.0;
inline double get_now() { return ((double)millis()) / 1000.0f; }


// Math for fading each pixel each tick towards another value.
uint8_t fade_lookup[64*64];
const float FADE_RATE = 0.9;
const uint8_t lower_five_mask = 0x1F;
const uint8_t lower_six_mask = 0x3F;
void populate_fade_lookup_table(){
  for (int32_t i = 0; i < 64; i++){
    for (int32_t j = 0; j < 64; j++){
      // Converge to the background (j) without getting stuck in a neighboring
      // value.
      float err = (i - j) * FADE_RATE;
      // This clamping value is a function of the fade rate and ultimate numeric
      // precision... but I just guess till the issue goes away.
      if (fabs(err) < 10){
        err = 0.0;
      }
      fade_lookup[i * 64 + j] = j + err;
    }
  }
}


void setup() {
  populate_fade_lookup_table();
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
  
  uint16_t * framebuffer = gfx->getFramebuffer();
  for (int i = 0; i < WIDTH*HEIGHT; i++){
    framebuffer[i] = nebula_bitmap[i];
  }
  
  sim->initialize_all_particles();
  start_t = get_now();
}

// === Main loop ===
int n_frames = 0;
double last_draw = 0.0;

void loop() {
  double t = get_now();

  sim->dynamics_update(t);

  if (t - last_draw >= DRAW_DT) {
    last_draw = t;
    n_frames++;
    double fps = n_frames / (t - start_t);
    Serial.printf("FPS: %.2f\n", fps);

    // We instead undraw previous pixels to save screen clearing time.
    // gfx->fillScreen(BLACK);

    uint16_t * framebuffer = gfx->getFramebuffer();
    // Render background of the nebula bitmap with fading star trails
    // for (int i = 0; i < WIDTH*HEIGHT; i++){
    //   uint16_t bg = nebula_bitmap[i];
    //   uint16_t x = framebuffer[i];
    //   uint16_t x_r = x & lower_five_mask;
    //   uint16_t x_g = (x >> 5) & lower_six_mask;
    //   uint16_t x_b = (x >> 11) & lower_five_mask;
    //   // note(gizatt) artificially darkening bg here
    //   uint16_t bg_r = bg & lower_five_mask >> 2;
    //   uint16_t bg_g = (bg >> 5) & lower_six_mask >> 2;
    //   uint16_t bg_b = (bg >> 11) & lower_five_mask >> 2;
    //   // This morphs the current value (x) towards the background (bg).
    //   uint16_t r = fade_lookup[x_r * 64 + bg_r];
    //   uint16_t g = fade_lookup[x_g * 64 + bg_g];
    //   uint16_t b = fade_lookup[x_b * 64 + bg_b];
    //   *(framebuffer + i) = r | (g << 5) | (b << 11);
    // }
    for (int i = 0; i < N_PARTICLES; ++i) {
      uint16_t u = sim->get_pixel_u(i);
      uint16_t v = sim->get_pixel_v(i);
      if (u < 0 || u >= WIDTH || v < 0 || v >= HEIGHT){
        continue;
      }
      // Undraw last position.
      gfx->writePixel(u, v, nebula_bitmap[v*HEIGHT+u]);
    }

    // And draw new ones.
    sim->update_drawing_info();
    for (int i = 0; i < N_PARTICLES; ++i) {
      uint16_t u = sim->get_pixel_u(i);
      uint16_t v = sim->get_pixel_v(i);
      if (u < 0 || u >= WIDTH || v < 0 || v >= HEIGHT){
        continue;
      }
      gfx->writePixel(u, v, sim->get_color(i));
    }
    gfx->flush();
  }
}
