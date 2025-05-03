#ifndef PARTICLE_SIM_H
#define PARTICLE_SIM_H

#include "esp_dsp.h"
#include <ArduinoEigenDense.h>
#include <array>
#include <cmath>
#include <iostream>
#include <random>

#include "nebula_bitmap.h"

#include "Config.h"

using Vec3f = Eigen::Vector3f;

uint16_t color565(uint8_t red, uint8_t green, uint8_t blue) {
  return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
}

float positions[N_PARTICLES * 3];
float velocities[N_PARTICLES * 3];
uint8_t colors[N_PARTICLES * 3];

class ParticleSim {
public:
  ParticleSim()
      : last_dynamics_update(-1.0), last_resampling_update(-1.0),
        m_gen(std::random_device()()), m_theta_dist(0, 0.05f),
        m_spiral_dist(0, N_SPIRALS - 1), m_r_dist(0.75f, 0.05f),
        m_z_dist(0, 0.05f), m_speed_variance(1.0f, INITIAL_VELOCITY_VARIANCE),
        world_Q_planar(Eigen::Matrix3f::Identity()) {
    initialize_all_particles();
    update_drawing_info();
    set_normal_vector({0.5f, 1.0f, 0.85f});
  }

  int16_t get_pixel_u(int index) const {
    return pixel_positions[index * 2 + 0];
  }
  int16_t get_pixel_v(int index) const {
    return pixel_positions[index * 2 + 1];
  }
  uint16_t get_color(int index) const {
    int i = index * 3;
    return color565(colors[i + 0], colors[i + 1], colors[i + 2]);
  }

  void set_normal_vector(const std::array<float, 3> &normal_vector) {
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

  void initialize_all_particles() {
    for (int i = 0; i < N_PARTICLES; ++i) {
      initialize_particle(i);
    }
  }

  void initialize_particle(int index) {
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

    // TODO: Sample
    colors[index * 3 + 0] =  255 - (index % 10);
    colors[index * 3 + 1] =  255 - (index % 20);
    colors[index * 3 + 2] =  255 - (index % 30);
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
      for (int i = 0; i < N_PARTICLES; i++){

        // Force computation 
        int index = i * 3;
        // Get these reads going all in parallel.
        p0 = positions[index + 0];
        p1 = positions[index + 1];
        p2 = positions[index + 2];
        v0 = velocities[index + 0];
        v1 = velocities[index + 1];
        v2 = velocities[index + 2];
        
        
        float r_norm_2 = p0*p0+p1*p1+p2*p2 + PADDING * PADDING;
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

        float speed_ratio = sqrt(v0*v0+v1*v1+v2*v2) / (MAX_SPEED * 0.5f);
        speed_ratio = std::max(std::min(speed_ratio, 1.0f), 0.0f);
        speed_ratio *= speed_ratio;
        // colors[index + 0] = 255 * 1.0f * speed_ratio;
        // colors[index + 1] = 255 * 0.5f * speed_ratio;
        // colors[index + 2] = 255 * (1.f - speed_ratio);
      }

      Serial.printf("Elapsed %ums\n", (micros() - now)/1000);

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
    for (int i = 0; i < N_PARTICLES; i++) {
      pixel_positions[i * 2 + 0] = (int16_t)(positions[i * 3 + 0] * 240 + 240);
      pixel_positions[i * 2 + 1] = (int16_t)(positions[i * 3 + 1] * 240 + 240);
    }
  }

private:

  int16_t pixel_positions[2 * N_PARTICLES];

  Eigen::Matrix3f world_Q_planar;
  Vec3f m_normal_vector;

  double last_dynamics_update;
  double last_resampling_update;

  std::mt19937 m_gen;
  std::normal_distribution<float> m_theta_dist;
  std::uniform_int_distribution<> m_spiral_dist;
  std::normal_distribution<float> m_r_dist;
  std::normal_distribution<float> m_z_dist;
  std::normal_distribution<float> m_speed_variance;
};

#endif // PARTICLE_SIM_H
