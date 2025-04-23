import numpy as np
import pygame
import time
from numba import njit

# Parameters
WIDTH, HEIGHT = 480, 480
N_PARTICLES = 5000
CENTER_PARTICLE_MASS = 500
G = 1e-5
DT = 0.05

# Particle arrays
pos = np.zeros((N_PARTICLES, 2), dtype=np.float32)
vel = np.zeros((N_PARTICLES, 2), dtype=np.float32)
force = np.zeros((N_PARTICLES, 2), dtype=np.float32)

@njit
def compute_brute_forces(pos, force, G):
    N = pos.shape[0]
    for i in range(N):
        fx, fy = 0.0, 0.0
        xi, yi = pos[i]
        for j in range(N):
            if i == j:
                continue
            dx = pos[j, 0] - xi
            dy = pos[j, 1] - yi
            r2 = dx * dx + dy * dy + 1e-4
            inv_r3 = 1.0 / (r2 * np.sqrt(r2))
            f = G * inv_r3
            fx += f * dx
            fy += f * dy
        force[i, 0] = fx
        force[i, 1] = fy

def init_particles():
    r = np.abs(np.random.normal(0.5, 0.25, N_PARTICLES))
    theta = np.random.uniform(0, 2 * np.pi, N_PARTICLES)
    pos[:, 0] = r * np.cos(theta)
    pos[:, 1] = r * np.sin(theta)
    v_mag = np.sqrt(G * CENTER_PARTICLE_MASS / r) * np.random.normal(1., 0.05, N_PARTICLES)
    vel[:, 0] = -v_mag * np.sin(theta)
    vel[:, 1] = v_mag * np.cos(theta)
    force[:] = 0

def dynamics_update():
    global pos, vel, force
    compute_brute_forces(pos, force, G)
    r_norm = np.linalg.norm(pos, axis=1, keepdims=True) + 0.1

    # Attract towards the center
    force -= G * CENTER_PARTICLE_MASS * pos / r_norm**3

    # Apply a torque to the particles to keep them in a circular orbit
    # force[:, 0] -= 0.001 * pos[:, 1] / r_norm.flatten()
    # force[:, 1] += 0.001 * pos[:, 0] / r_norm.flatten()

    #force += -pos * np.clip(r_norm - 0.6, 0, None) / r_norm
    #force -= vel * np.clip(np.linalg.norm(vel, axis=1, keepdims=True) - 0.5, 0, None) / np.linalg.norm(vel, axis=1, keepdims=True)
    

    vel[:] += force * DT
    pos[:] += vel * DT

    needs_reset = np.logical_or(np.linalg.norm(pos, axis=1) > 1.0, np.linalg.norm(vel, axis=1) > 0.5)
    #needs_reset = np.logical_or(needs_reset, np.linalg.norm(pos, axis=1) < 0.02)
    pos[needs_reset] = np.random.normal(0, 0.25, (np.sum(needs_reset), 2))
    rnorm = np.linalg.norm(pos[needs_reset], axis=1) + 0.1
    vel[needs_reset, 0] = -pos[needs_reset, 1] / rnorm * 0.4
    vel[needs_reset, 1] = pos[needs_reset, 0] / rnorm * 0.4


def draw_particles():
    speeds = np.linalg.norm(vel, axis=1)
    vmax = np.percentile(speeds, 95) + 1e-5
    norm_speeds = np.clip(speeds / vmax, 0, 1)

    rgb = np.zeros((N_PARTICLES, 3), dtype=np.uint8)
    rgb[:, 0] = (127  + 127 * norm_speeds).astype(np.uint8)
    rgb[:, 1] = (255 * (1 - norm_speeds)).astype(np.uint8)
    rgb[:, 2] = (255 * (1 - 0.7 * norm_speeds)).astype(np.uint8)

    screen_xy = np.floor((pos + 1) * WIDTH // 2).astype(np.int32)
    img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

    mask = (screen_xy[:, 0] >= 0) & (screen_xy[:, 0] < WIDTH) & \
           (screen_xy[:, 1] >= 0) & (screen_xy[:, 1] < HEIGHT)
    img[screen_xy[mask, 1], screen_xy[mask, 0]] = rgb[mask]

    pygame.surfarray.blit_array(screen, img.swapaxes(0, 1))
    pygame.display.flip()

# Main loop
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

running = True
init_particles()
while running:
    frame_start = time.time()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_BACKSPACE:
            init_particles()

    t0 = time.time()
    dynamics_update()
    t1 = time.time()
    draw_particles()
    t2 = time.time()

    print(f"Dynamics: {(t1 - t0)*1000:.1f} ms | Draw: {(t2 - t1)*1000:.1f} ms | Total: {(t2 - frame_start)*1000:.1f} ms")

    clock.tick(60)

pygame.quit()
