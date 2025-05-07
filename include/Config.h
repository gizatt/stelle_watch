#pragma once

// Screen size
const int WIDTH = 480;
const int HEIGHT = 480;
const double DRAW_DT = 1.0 / 20.0;
const double RESAMPLING_DT = 0.05;

// Sim parameters
const int N_PARTICLES = 5000;
// Particles have unit mass.
const float CENTER_MASS = 1000.0;
const float G = 5e-4;
const double DYNAMICS_DT = 0.03;
const float REAL_TIME_FACTOR = 1.0;
const float PADDING = 0.05;
const float VELOCITY_DAMPING = 0.3;
const float INITIAL_VELOCITY_PENALTY = 0.9;
const float INITIAL_VELOCITY_VARIANCE = 0.05;
const float MAX_SPEED = 3.5;
const int N_SPIRALS = 5;
const int MAX_AGE_MS = 2500;