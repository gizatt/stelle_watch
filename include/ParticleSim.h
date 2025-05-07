#ifndef PARTICLE_SIM_H
#define PARTICLE_SIM_H

#include "esp_dsp.h"
#include <ArduinoEigenDense.h>
#include <array>
#include <cmath>
#include <iostream>
#include <random>

#include "Config.h"
#include "hsv2rgb.h"
#include "nebula_bitmap.h"
#include "rgb565_math.h"

using Vec3f = Eigen::Vector3f;

uint16_t color565(uint8_t red, uint8_t green, uint8_t blue) {
  return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
}

inline int uv_to_index(int u, int v, int width) { return u * width + v; }

// These are defined statically so they get put in DRAM, which seems empirically
// much faster.
DRAM_ATTR float positions[N_PARTICLES * 3];
DRAM_ATTR float velocities[N_PARTICLES * 3];
DRAM_ATTR uint8_t radii[N_PARTICLES];
DRAM_ATTR uint8_t colors[N_PARTICLES * 3];
// UV and projection depth Z.
DRAM_ATTR int16_t pixel_positions[3 * N_PARTICLES];

int16_t ages_ms[N_PARTICLES];

// Pregenerated sprites of each radius up to the max radius. Sprite at index i
// is size (ind*2+1)^2.
constexpr int MAX_RADIUS = 6;
uint8_t *sprites[MAX_RADIUS + 1];

uint16_t *nebula_psram;

class ParticleSim {
public:
  ParticleSim()
      : last_dynamics_update(-1.0), last_resampling_update(-1.0),
        m_gen(std::random_device()()), m_theta_dist(0, 0.05f),
        m_spiral_dist(0, N_SPIRALS - 1), m_r_dist(0.75f, 0.05f),
        m_z_dist(0, 0.05f), m_speed_variance(1.0f, INITIAL_VELOCITY_VARIANCE),
        m_out_of_plane_speed_dist(0.0, 0.01),
        world_Q_planar(Eigen::Matrix3f::Identity()), m_hue_dist(0.6, 0.9),
        m_saturation_dist(0.0, 0.4), m_value_dist(0.2, 1.0), m_radii_dist(0.5),
        m_focal_length(WIDTH / 2.0) {
    initialize_sprites();
    set_normal_vector({0.5f, 1.0f, 0.85f});
    // Calculate projection matrix and info.
    m_view_center << WIDTH / 2.0, HEIGHT / 2.0, 0.0;
    set_view_direction(Vec3f(0.0, 0.0, 1.0));

    do_onetime_particle_init();
    update_drawing_info();

    nebula_psram = (uint16_t *)ps_malloc(sizeof(nebula_bitmap));
    memcpy(nebula_psram, nebula_bitmap, sizeof(nebula_bitmap));


  }

  void initialize_sprites() {
    // Initializes each sprite to contain a gaussian kernel.
    for (int radius = 0; radius < MAX_RADIUS + 1; radius++) {
      int side_length = radius * 2 + 1;
      uint8_t *sprite =
          (uint8_t *)ps_malloc(side_length * side_length * sizeof(uint8_t));
      sprites[radius] = sprite;
      for (int u = 0; u < radius + 1; u++) {
        for (int v = 0; v < radius + 1; v++) {
          uint8_t value = exp(-(u * u + v * v) / radius) * 255;
          sprite[uv_to_index(radius + u, radius + v, side_length)] = value;
          sprite[uv_to_index(radius + u, radius - v, side_length)] = value;
          sprite[uv_to_index(radius - u, radius + v, side_length)] = value;
          sprite[uv_to_index(radius - u, radius - v, side_length)] = value;
        }
      }
    }
  }

  int16_t get_pixel_u(int index) const {
    return pixel_positions[index * 3 + 0];
  }
  int16_t get_pixel_v(int index) const {
    return pixel_positions[index * 3 + 1];
  }
  int16_t get_pixel_z(int index) const {
    return pixel_positions[index * 3 + 2];
  }
  uint16_t get_color(int index) const {
    int i = index * 3;
    return color565(colors[i + 0], colors[i + 1], colors[i + 2]);
  }

  void set_normal_vector(const std::array<float, 3> &normal_vector) {
    // Set the "galaxy" normal vector for particle spawning purposes.
    Vec3f normal = Vec3f(normal_vector.data()).normalized();
    m_normal_vector = normal;

    Vec3f x_axis(1, 0, 0);
    if (x_axis.dot(normal) > 0.99f) {
      x_axis = Vec3f(0, 1, 0);
    }
    Vec3f y_axis = normal.cross(x_axis).normalized();
    x_axis = y_axis.cross(normal).normalized();

    world_Q_planar.col(0) = x_axis;
    world_Q_planar.col(1) = y_axis;
    world_Q_planar.col(2) = normal;
  }

  void do_onetime_particle_init() {
    for (int i = 0; i < N_PARTICLES; ++i) {
      initialize_particle_position(i);
      initialize_particle_unique_properties(i);
    }
  }

  void initialize_particle_position(int index) {
    float theta =
        m_theta_dist(m_gen) + m_spiral_dist(m_gen) * (2 * M_PI / N_SPIRALS);
    float r = std::abs(m_r_dist(m_gen));
    Vec3f uv;
    Vec3f uv_dot;

    uv(0) = r * std::cos(theta);
    uv(1) = r * std::sin(theta);
    uv(2) = m_z_dist(m_gen);

    float speed = INITIAL_VELOCITY_PENALTY * std::sqrt(G * CENTER_MASS / r) *
                  m_speed_variance(m_gen);
    uv_dot(0) = -speed * std::sin(theta);
    uv_dot(1) = speed * std::cos(theta);
    uv_dot(2) = m_out_of_plane_speed_dist(m_gen);

    const auto pos = world_Q_planar * uv;
    const auto vel = world_Q_planar * uv_dot;
    for (int i = 0; i < 3; i++) {
      positions[index * 3 + i] = pos(i);
      velocities[index * 3 + i] = vel(i);
    }
  }
  void initialize_particle_unique_properties(int index) {
    // Initialize a random color.
    float h = m_hue_dist(m_gen);
    float s = m_value_dist(m_gen);
    float v = m_saturation_dist(m_gen);
    hsv2rgb(h, s, v, colors + index * 3);

    // Initialize its radius from a clipped geometric distribution. The
    // geometric distribution support is on [1, inf], so by clipping and then
    // subtracting 1, we get support on [0, MAX_RADIUS].
    int radius = m_radii_dist(m_gen);
    radii[index] = min(max(radius, 1), MAX_RADIUS + 1) - 1;

    // Distribute particle ages evenly among all particles.
    ages_ms[index] = ((float)index / N_PARTICLES) * MAX_AGE_MS;
  }

  void dynamics_update(double t) {
    // Catch the startup condition.
    if (last_dynamics_update <= 0.0) {
      last_dynamics_update = t;
    }

    if (t - last_dynamics_update > DYNAMICS_DT) {
      float dt = min(t - last_dynamics_update, DYNAMICS_DT);
      last_dynamics_update = t;

      // This is terribly ugly but can I convince it to do no memory
      // reads/writes?
      float p0, p1, p2;
      float v0, v1, v2;
      for (int i = 0; i < N_PARTICLES; i++) {

        // Force computation
        int index = i * 3;
        // Get these reads going all in parallel.
        p0 = positions[index + 0];
        p1 = positions[index + 1];
        p2 = positions[index + 2];
        v0 = velocities[index + 0];
        v1 = velocities[index + 1];
        v2 = velocities[index + 2];

        float r_norm_2 = p0 * p0 + p1 * p1 + p2 * p2 + PADDING * PADDING;
        float force_scaling = -G * CENTER_MASS / (sqrt(r_norm_2) * r_norm_2);

        // Integration math
        float force = force_scaling * p0;
        v0 += dt * (force - v0 * VELOCITY_DAMPING);
        p0 += dt * v0;
        velocities[index + 0] = v0;
        positions[index + 0] = p0;
        force = force_scaling * p1;
        v1 += dt  * (force - v1 * VELOCITY_DAMPING);
        p1 += dt * v1;
        velocities[index + 1] = v1;
        positions[index + 1] = p1;
        force = force_scaling * p2;
        v2 += dt  * (force - v2 * VELOCITY_DAMPING);
        p2 += dt * v2;
        velocities[index + 2] = v2;
        positions[index + 2] = p2;
        ages_ms[i] += dt * 1000;
      }

      if (t - last_resampling_update > RESAMPLING_DT) {
        last_resampling_update = t;
        for (int i = 0; i < N_PARTICLES; ++i) {
          if (ages_ms[i] >= MAX_AGE_MS) {
            initialize_particle_position(i);
            ages_ms[i] = 0;
          }
        }
      }
    }
  }

  void update_drawing_info() {
    // Right now, naive 2d projection
    Eigen::Matrix3f K = m_R * m_focal_length;
    for (int i = 0; i < N_PARTICLES; i++) {
      int particle_index = i * 3;
      for (int j = 0; j < 3; j++) {
        pixel_positions[particle_index + j] = m_view_center[j];
        for (int k = 0; k < 3; k++) {
          pixel_positions[particle_index + j] +=
              K(j, k) * positions[particle_index + k];
        }
      }
    }
  }

  void undraw_particles(uint16_t *framebuffer) {
    int pixel_index = 0;
    int n_accesses = 0;
    // For large numbers of particles, particularly
    // with a lot of overlap, it's faster to just copy the background image
    // wholesale.
    memcpy(framebuffer, nebula_psram, sizeof(nebula_bitmap));
    // for (int i = 0; i < N_PARTICLES; i++) {
    //   int radius = radii[i];
    //   int side_length = radius * 2 + 1;
    //   uint16_t u_center = pixel_positions[pixel_index + 0];
    //   uint16_t v_center = pixel_positions[pixel_index + 1];
    //   pixel_index += 3;
    //   for (int du = 0; du < side_length; du++) {
    //     uint16_t u = u_center + du - radius;
    //     if (u < 0 || u >= WIDTH) {
    //       continue;
    //     }
    //     for (int dv = 0; dv < side_length; dv++) {
    //       uint16_t v = v_center + dv - radius;
    //       if (v < 0 || v >= HEIGHT) {
    //         continue;
    //       }
    //       int out_index = uv_to_index(u, v, WIDTH);
    //       framebuffer[out_index] = 0;// nebula_bitmap[out_index];
    //       n_accesses ++;
    //     }
    //   }
    // }
    // Serial.printf("%d accesses in undraw.\n", n_accesses);
  }

  void draw_particles(uint16_t *framebuffer) {
    int pixel_index = 0;

    uint8_t fast_r = 1.0*255;
    uint8_t fast_g = 0.7*255;
    uint8_t fast_b = 0.5*255;
      

    for (int i = 0; i < N_PARTICLES; i++) {
      float v0 = velocities[pixel_index + 0];
      float v1 = velocities[pixel_index + 1];
      float v2 = velocities[pixel_index + 2];
      float speed_ratio = sqrt(v0 * v0 + v1 * v1 + v2 * v2) / (MAX_SPEED * 0.75f);
      speed_ratio = std::max(std::min(speed_ratio, 1.0f), 0.0f);
      speed_ratio *= speed_ratio;
      
      int radius = radii[i];
      int side_length = radius * 2 + 1;
      uint8_t * sprite = sprites[radius];
      uint8_t new_r = colors[pixel_index] * (1. - speed_ratio) + fast_r * speed_ratio;
      uint8_t new_g = colors[pixel_index + 1] * (1. - speed_ratio) + fast_g * speed_ratio;
      uint8_t new_b = colors[pixel_index + 2] * (1. - speed_ratio) + fast_b * speed_ratio;
      uint16_t new_rgb = RGB565(new_r, new_g, new_b);
      uint16_t u_center = pixel_positions[pixel_index + 0];
      uint16_t v_center = pixel_positions[pixel_index + 1];
      pixel_index += 3;
      int sprite_index = 0;
      // framebuffer[uv_to_index(u_center, v_center, WIDTH)] = RGB565(255, 255,
      // 255); continue;
      for (int du = -radius; du < radius+1; du++) {
        uint16_t u = u_center + du;
        if (u < 0 || u >= WIDTH) {
          continue;
        }
        for (int dv = -radius; dv < radius; dv++) {
          sprite_index++;
          uint16_t v = v_center + dv;
          if (v < 0 || v >= HEIGHT) {
            continue;
          }
          if ((du * du + dv * dv) >= radius*radius) {
            continue;
          }
          int out_index = uv_to_index(u, v, WIDTH);
          uint8_t alpha = sprite[sprite_index - 1];
          uint16_t old_val = framebuffer[out_index];

          // Correct but slow
          //uint8_t beta = (255 - alpha) >> 2;
          // uint8_t old_r = ((old_val >> 11) & 0x1F) << 3;  // 5 bits → 8 bits
          // uint8_t old_g = ((old_val >> 5) & 0x3F) << 2;   // 6 bits → 8 bits
          // uint8_t old_b = (old_val & 0x1F) << 3;          // 5 bits → 8 bits
          // uint8_t blended_r = (alpha * new_r + beta * old_r) >> 8;
          // uint8_t blended_g = (alpha * new_g + beta * old_g) >> 8;
          // uint8_t blended_b = (alpha * new_b + beta * old_b) >> 8;
          // if (blended_r < old_r) blended_r = old_r;
          // if (blended_g < old_g) blended_g = old_g;
          // if (blended_b < old_b) blended_b = old_b;
          // framebuffer[out_index] = RGB565(blended_r, blended_g, blended_b);
          
          // Doesn't do additive brightness quite like I want
          // framebuffer[out_index] = alphablend(new_rgb, old_val, alpha, 0);

          // Doesn't incorporate alpha, just does circles. Kinda oversaturated
          // but cool bloomy-y effect.
          uint8_t old_r = ((old_val >> 11) & 0x1F) << 3;  // 5 bits → 8 bits
          uint8_t old_g = ((old_val >> 5) & 0x3F) << 2;   // 6 bits → 8 bits
          uint8_t old_b = (old_val & 0x1F) << 3;          // 5 bits → 8 bits
          framebuffer[out_index] = RGB565(min(old_r + new_r, 255), min(old_g + new_g, 255), min(old_b + new_b, 255));
        }
      }
    }
  }

  void set_view_direction(Vec3f view_direction) {
    // Set R from a view ray.
    view_direction /= view_direction.norm();
    Vec3f x_axis(1.0, 0.0, 0.0);
    if (x_axis.dot(view_direction) > 0.99) {
      x_axis << 0.0, 1.0, 0.0;
    }
    Vec3f y_axis = view_direction.cross(x_axis);
    x_axis = y_axis.cross(view_direction);
    Vec3f z_axis = x_axis.cross(y_axis);
    m_R.row(0) = x_axis;
    m_R.row(1) = y_axis;
    m_R.row(2) = z_axis;
  }

private:
  Eigen::Matrix3f world_Q_planar;
  Vec3f m_normal_vector;

  float m_focal_length;
  Vec3f m_view_center;
  Eigen::Matrix3f m_R; // View direction as a rotation matrix.

  double last_dynamics_update;
  double last_resampling_update;

  std::mt19937 m_gen;
  std::normal_distribution<float> m_theta_dist;
  std::uniform_int_distribution<> m_spiral_dist;
  std::normal_distribution<float> m_r_dist;
  std::normal_distribution<float> m_z_dist;
  std::normal_distribution<float> m_speed_variance;
  std::normal_distribution<float> m_out_of_plane_speed_dist;
  std::uniform_real_distribution<float> m_hue_dist;
  std::uniform_real_distribution<float> m_value_dist;
  std::uniform_real_distribution<float> m_saturation_dist;
  std::geometric_distribution<> m_radii_dist;
};

#endif // PARTICLE_SIM_H
