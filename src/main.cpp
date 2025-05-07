#include "DisplayMagic.h"
#include "ParticleSim.h"
#include <Arduino_GFX_Library.h>
#include <math.h>
#include "nebula_bitmap.h"
#include "TAMC_GT911.h"

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


#define TOUCH_SDA  GPIO_NUM_15
#define TOUCH_SCL  GPIO_NUM_7
#define TOUCH_INT GPIO_NUM_16
#define TOUCH_RST 1 // unused, so lying to the library about it
#define TOUCH_WIDTH  WIDTH
#define TOUCH_HEIGHT HEIGHT

TAMC_GT911 tp = TAMC_GT911(TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST, TOUCH_WIDTH, TOUCH_HEIGHT);


ParticleSim *sim = new ParticleSim();

double start_t = 0.0;
inline double get_now() { return ((double)millis()) / 1000.0f; }


void setup() {
  Serial.begin(115200);

  pinMode(GPIO_NUM_4, INPUT);
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

  tp.begin(0x5D);
  tp.setRotation(ROTATION_LEFT);
  
  uint16_t * framebuffer = gfx->getFramebuffer();
  for (int i = 0; i < WIDTH*HEIGHT; i++){
    framebuffer[i] = nebula_bitmap[i];
  }
  start_t = get_now();
}

// === Main loop ===
int n_frames = 0;
double last_draw = 0.0;
double last_read_tp = 0.0;
bool needs_text_cleared = false;

void loop() {
  double t = get_now();

  if (t - last_read_tp >= 0.1){
    tp.read();
    if (tp.isTouched && tp.touches >= 1){
      // Set gravity to first touch point
      sim->set_secondary_gravity_center_from_screen_coords(tp.points[0].x, tp.points[0].y);
      sim->set_secondary_gravity_scaling(2.0);
      for (int i=0; i<tp.touches; i++){
        Serial.print("Touch ");Serial.print(i+1);Serial.print(": ");;
        Serial.print("  x: ");Serial.print(tp.points[i].x);
        Serial.print("  y: ");Serial.print(tp.points[i].y);
        Serial.print("  size: ");Serial.println(tp.points[i].size);
        Serial.println(' ');
      }
    } else {
      // Reset gravity back to center.
      // sim->set_gravity_center_from_screen_coords(WIDTH/2, HEIGHT/2);
      sim->set_secondary_gravity_scaling(0.0);
    }
  }

  sim->dynamics_update(t);

  if (t - last_draw >= DRAW_DT) {
    last_draw = t;
    n_frames++;
    double fps = n_frames / (t - start_t);
    Serial.printf("FPS: %.2f\n", fps);

    // Draw text
    uint16_t * framebuffer = gfx->getFramebuffer();
    if (t <= 10.0){
      gfx->setRotation(3);
      gfx->setCursor(WIDTH/2, 25);
      gfx->printf("~~xen 2025~~");
      gfx->setCursor(WIDTH/2, 35);
      gfx->printf("battery: %0.2f v", analogReadMilliVolts(GPIO_NUM_4)*2. / 1000.);
      needs_text_cleared = true;
    } else if (needs_text_cleared){
      // Remove remnant text by redrawing whole screen once.
      for (int i = 0; i < WIDTH*HEIGHT; i++){
        framebuffer[i] = nebula_bitmap[i];
      }
      needs_text_cleared = false;
    }

    // Undraw previous pixels to save screen clearing time.
    auto t_start = micros();
    sim->undraw_particles(framebuffer);
    auto t_undraw = micros();
    // And draw new ones.
    sim->update_drawing_info();
    auto t_projection = micros();
    sim->draw_particles(framebuffer);
    auto t_draw = micros();

    // Debug: draw touch poitns
    /*
    if (tp.isTouched && tp.touches >= 1){
      for (int i=0; i<tp.touches; i++){
        gfx->drawCircle(tp.points[i].x, tp.points[i].y, 10, RGB565(255, 255, 255));
      }
    }
    */

    gfx->flush();
    auto t_flush = micros();
    Serial.printf("Undraw %d, Project %d, Draw %d, Flush %d\n", t_undraw - t_start, t_projection - t_undraw, t_draw - t_projection, t_flush - t_draw);
  }
}
