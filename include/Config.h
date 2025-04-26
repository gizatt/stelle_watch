#pragma once

// Screen size
const int WIDTH = 480;
const int HEIGHT = 480;
const double DRAW_DT = 1.0 / 30.0;

// Sim parameters
const int N_PARTICLES = 9000;
// Particles have unit mass.
const float CENTER_MASS = 1000.0;
const float G = 1e-4;
const double DYNAMICS_DT = 0.02;
const double RESAMPLING_DT = 0.25;
const float REAL_TIME_FACTOR = 1.0;
const float PADDING = 1e-4;
const float VELOCITY_DAMPING = 0.01;
const float INITIAL_VELOCITY_PENALTY = 0.5;
const float INITIAL_VELOCITY_VARIANCE = 0.1;
const float MIN_DISTANCE = 0.02;
const float MAX_SPEED = 2.0;
const int EXPECTED_LIFTIME = 5;
const int N_SPIRALS = 6;