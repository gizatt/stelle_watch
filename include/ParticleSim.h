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

using Vec3f = Eigen::Vector3f;

uint16_t color565(uint8_t red, uint8_t green, uint8_t blue) {
  return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
}

inline int uv_to_index(int u, int v, int width) { return u * width + v; }

// These are defined statically so they get put in DRAM, which seems empirically
// much faster.
float positions[N_PARTICLES * 3];
float velocities[N_PARTICLES * 3];
uint8_t radii[N_PARTICLES];
uint8_t colors[N_PARTICLES * 3];
// UV and projection depth Z.
int16_t pixel_positions[3 * N_PARTICLES];

int16_t ages_ms[N_PARTICLES];

// Pregenerated sprites of each radius up to the max radius. Sprite at index i
// is size (ind*2+1)^2.
constexpr int MAX_RADIUS = 20;
uint8_t *sprites[MAX_RADIUS + 1];

class ParticleSim {
public:
  ParticleSim()
      : last_dynamics_update(-1.0), last_resampling_update(-1.0),
        m_gen(std::random_device()()), m_theta_dist(0, 0.05f),
        m_spiral_dist(0, N_SPIRALS - 1), m_r_dist(0.75f, 0.05f),
        m_z_dist(0, 0.05f), m_speed_variance(1.0f, INITIAL_VELOCITY_VARIANCE),
        world_Q_planar(Eigen::Matrix3f::Identity()), m_hue_dist(0.6, 0.9),
        m_saturation_dist(0.0, 0.4), m_value_dist(0.2, 1.0), m_radii_dist(0.2),
        m_focal_length(WIDTH / 2.0) {
    initialize_sprites();
    do_onetime_particle_init();

    update_drawing_info();
    set_normal_vector({0.5f, 1.0f, 0.85f});

    // Calculate projection matrix and info.
    m_view_center << WIDTH / 2.0, HEIGHT / 2.0, 0.0;
    set_view_direction(Vec3f(0.0, 0.0, 1.0));
  }

  void initialize_sprites() {
    // Initializes each sprite to contain a gaussian kernel.
    for (int radius = 0; radius < MAX_RADIUS + 1; radius++) {
      int side_length = radius * 2 + 1;
      uint8_t *sprite =
          (uint8_t *)malloc(side_length * side_length * sizeof(uint8_t));
      sprites[radius] = sprite;
      for (int u = 0; u < radius; u++) {
        for (int v = 0; v < radius; v++) {
          uint8_t value = exp(-u * u + v * v / radius) * 255;
          sprite[uv_to_index(u + radius, v + radius, side_length)] = value;
          sprite[uv_to_index(u - radius, v + radius, side_length)] = value;
          sprite[uv_to_index(u + radius, v - radius, side_length)] = value;
          sprite[uv_to_index(u - radius, v - radius, side_length)] = value;
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
    uv_dot(2) = 0.0; // Could have random velocity here too.

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
      unsigned long now = micros();
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
        v0 += dt * force;
        p0 += dt * v0;
        velocities[index + 0] = v0;
        positions[index + 0] = p0;
        force = force_scaling * p1;
        v1 += dt * force;
        p1 += dt * v1;
        velocities[index + 1] = v1;
        positions[index + 1] = p1;
        force = force_scaling * p2;
        v2 += dt * force;
        p2 += dt * v2;
        velocities[index + 2] = v2;
        positions[index + 2] = p2;

        float speed_ratio =
            sqrt(v0 * v0 + v1 * v1 + v2 * v2) / (MAX_SPEED * 0.5f);
        speed_ratio = std::max(std::min(speed_ratio, 1.0f), 0.0f);
        speed_ratio *= speed_ratio;
        // colors[index + 0] = 255 * 1.0f * speed_ratio;
        // colors[index + 1] = 255 * 0.5f * speed_ratio;
        // colors[index + 2] = 255 * (1.f - speed_ratio);
      }

      Serial.printf("Elapsed %ums\n", (micros() - now) / 1000);

      // if (t - last_resampling_update > RESAMPLING_DT) {
      //   last_resampling_update = t;
      //   for (int i = 0; i < N_PARTICLES; ++i) {
      //     float norm_squared = 0.0;
      //     dsps_dotprod_f32_aes3(positions + 3 * i, positions + 3 * i,
      //                           &norm_squared, 3);
      //     float speed_squared = 0.0;
      //     dsps_dotprod_f32_aes3(velocities + 3 * i, velocities + 3 * i,
      //                           &speed_squared, 3);
      //     if ((norm_squared > 4.0f) ||
      //         (norm_squared < MIN_DISTANCE * MIN_DISTANCE) ||
      //         (speed_squared > MAX_SPEED * MAX_SPEED)) {
      //       initialize_particle(i);
      //     }
      //   }
      // }
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
    for (int i = 0; i < N_PARTICLES; ++i) {
      uint16_t u = pixel_positions[pixel_index + 0];
      uint16_t v = pixel_positions[pixel_index + 1];
      pixel_index += 3;
      if (u < 0 || u >= WIDTH || v < 0 || v >= HEIGHT) {
        continue;
      }
      int out_index = uv_to_index(u, v, WIDTH);
      framebuffer[out_index] = nebula_bitmap[out_index];
    }
  }

  void draw_particles(uint16_t *framebuffer) {
    int pixel_index = 0;
    for (int i = 0; i < N_PARTICLES; ++i) {
      uint16_t u = pixel_positions[pixel_index + 0];
      uint16_t v = pixel_positions[pixel_index + 1];
      pixel_index += 3;
      if (u < 0 || u >= WIDTH || v < 0 || v >= HEIGHT) {
        continue;
      }
      framebuffer[uv_to_index(u, v, WIDTH)] = get_color(i);
    }
  }

  void set_view_direction(Vec3f view_direction) {
    // Set R from a view ray.
    view_direction /= view_direction.norm();
    Vec3f x_axis(1.0, 0.0, 0.0);
    if (x_axis.dot(view_direction) > 0.99){
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
  std::uniform_real_distribution<float> m_hue_dist;
  std::uniform_real_distribution<float> m_value_dist;
  std::uniform_real_distribution<float> m_saturation_dist;
  std::geometric_distribution<> m_radii_dist;
};

#endif // PARTICLE_SIM_H
