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

    // Undraw previous pixels to save screen clearing time.
    uint16_t * framebuffer = gfx->getFramebuffer();
    sim->undraw_particles(framebuffer);
    // And draw new ones.
    sim->update_drawing_info();
    sim->draw_particles(framebuffer);
    gfx->flush();
  }
}
