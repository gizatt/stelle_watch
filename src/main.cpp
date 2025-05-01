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

double start_t = 0.0;
inline double get_now() { return ((double)millis()) / 1000.0f; }


// Math for fading each pixel each tick.
// Precomputed lookup table of darkenings for up to 6 bit numbers
uint16_t darkenings[64];
const float DARKENING_RATIO = 0.95;
void populate_darkenings(){
  for (int i = 0; i < 64; i++){
    darkenings[i] = i * DARKENING_RATIO;
  }
}
const uint8_t lower_five_mask = 0x1F;
const uint8_t lower_six_mask = 0x3F;
inline void darken(uint16_t * value){
  *value = 
  darkenings[(*value) & lower_five_mask ] |
  darkenings[(*value >> 5) & lower_six_mask] << 5 |
  darkenings[(*value >> 5+6) & lower_five_mask] << (5+6);
}


void setup() {
  populate_darkenings();
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
  start_t = get_now();
}

// === Main loop ===
int n_frames = 0;
double last_draw = 0.0;

inline void draw_pos(uint16_t u, uint16_t v, uint16_t color) {
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

  if (t - last_draw >= DRAW_DT) {
    last_draw = t;
    n_frames++;
    double fps = n_frames / (t - start_t);
    Serial.printf("FPS: %.2f\n", fps);

    // We instead undraw previous pixels to save screen clearing time.
    // gfx->fillScreen(BLACK);

    uint16_t * framebuffer = gfx->getFramebuffer();
    for (int i = 0; i < WIDTH*HEIGHT; i++){
      darken(framebuffer + i);
    }
    // for (int i = 0; i < N_PARTICLES; ++i) {
    //   // Undraw last position.
    //    draw_pos(sim->get_pixel_u(i), sim->get_pixel_v(i), BLACK);
    // }

    // And draw new ones.
    sim->update_drawing_info();
    for (int i = 0; i < N_PARTICLES; ++i) {
       draw_pos(sim->get_pixel_u(i), sim->get_pixel_v(i), sim->get_color(i));
    }
    gfx->flush();
  }
}
