#pragma once

// Screen size
const int WIDTH = 480;
const int HEIGHT = 480;
const double DRAW_DT = 1.0 / 24.0;

// Sim parameters
const int N_PARTICLES = 10000;
// Particles have unit mass.
const double CENTER_MASS = 1000.0;
const double G = 1e-4;
const double DYNAMICS_DT = 0.05;
const double INTEGRATION_DT = 0.02;
const double RESAMPLING_DT = 0.25;
const double REAL_TIME_FACTOR = 1.0;
const double PADDING = 1e-4;
const double VELOCITY_DAMPING = 0.01;
const double INITIAL_VELOCITY_PENALTY = 0.5;
const double INITIAL_VELOCITY_VARIANCE = 0.1;
const double MIN_DISTANCE = 0.02;
const double MAX_SPEED = 2.0;
const int EXPECTED_LIFTIME = 5;
const int N_SPIRALS = 6;