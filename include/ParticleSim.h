#ifndef PARTICLE_SIM_H
#define PARTICLE_SIM_H

#include <ArduinoEigenDense.h>
#include <array>
#include <cmath>
#include <iostream>
#include <random>

#include "Config.h"

using Vec3f = Eigen::Vector3f;

// We use this type to store positions, velocities, and forces. Ideally, this would
// be statically sized, but that triggers a stack allocation, and we run out of stack memory
// on ESP32. So we allocate *once* on the heap at startup instead.
using Mat3xNf = Eigen::Matrix<float, 3, -1>;

class ParticleSim {
public:
    ParticleSim() : last_dynamics_update(0.0), last_integration_update(0.0), m_gen(std::random_device()()),
                                    m_theta_dist(0, 0.05f), m_spiral_dist(0, N_SPIRALS - 1),
                                    m_r_dist(0.75f, 0.05f), m_z_dist(0, 0.05f),
                                    m_speed_variance(1.0f, INITIAL_VELOCITY_VARIANCE) {
        positions.resize(3, N_PARTICLES);
        velocities.resize(3, N_PARTICLES);
        forces.resize(3, N_PARTICLES);
        colors.resize(3, N_PARTICLES);
        initialize_all_particles();
        set_normal_vector({0.5f, 1.0f, 0.85f});
    }

    const Mat3xNf &get_positions() const { return positions; }
    const Mat3xNf &get_velocities() const { return velocities; }
    const Mat3xNf &get_forces() const { return forces; }
    const Eigen::Matrix<uint8_t, 3, -1> &get_colors() const { return colors; }

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
        uv_dot(2) = m_z_dist(m_gen);

        positions.col(index) = world_Q_planar * uv;
        velocities.col(index) = world_Q_planar * uv_dot;

        // TODO: Sample
        colors(0, index) = 255;
        colors(1, index) = 255;
        colors(2, index) = 255;
    }

    void dynamics_update(double t) {

        if (t - last_dynamics_update > DYNAMICS_DT){
            last_dynamics_update = t;
            forces.setZero();
            const float inv_padding = 1.0f / PADDING;
            for (int i = 0; i < N_PARTICLES; ++i) {
                float r_norm = positions.col(i).squaredNorm() + PADDING * PADDING;
                float inv_r_norm_cubed = 1.0f / (std::sqrt(r_norm) * r_norm);
                forces.col(i).noalias() = -G * CENTER_MASS * positions.col(i) * inv_r_norm_cubed;
                // forces.col(i).noalias() -= velocities.col(i) * VELOCITY_DAMPING; // Ignored for speed?
            }
        }

        if (t - last_integration_update > INTEGRATION_DT){
            double dt = t - last_integration_update;
            last_integration_update = t;
            velocities.noalias() += forces * static_cast<float>(dt);
            positions.noalias() += velocities * static_cast<float>(dt);
        }

        if (t - last_resampling_update > RESAMPLING_DT){
            last_resampling_update = t;
            for (int i = 0; i < N_PARTICLES; ++i) {
                float norm_squared = positions.col(i).squaredNorm();
                float speed_squared = velocities.col(i).squaredNorm();
                if ((norm_squared > 4.0f) || (norm_squared < MIN_DISTANCE * MIN_DISTANCE) || 
                (speed_squared > MAX_SPEED * MAX_SPEED)) {
                initialize_particle(i);
                }
            }
        }
    }

private:
    Mat3xNf positions;
    Mat3xNf velocities;
    Mat3xNf forces;
    Eigen::Matrix<uint8_t, 3, -1> colors;
    Eigen::Matrix3f world_Q_planar;
    Vec3f m_normal_vector;
    double last_dynamics_update;
    double last_integration_update;
    double last_resampling_update;

    std::mt19937 m_gen;
    std::normal_distribution<float> m_theta_dist;
    std::uniform_int_distribution<> m_spiral_dist;
    std::normal_distribution<float> m_r_dist;
    std::normal_distribution<float> m_z_dist;
    std::normal_distribution<float> m_speed_variance;
};

#endif // PARTICLE_SIM_H
