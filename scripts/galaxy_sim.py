import pygame
import time
import numpy as np

import matplotlib.pyplot as plt

# Parameters
WIDTH, HEIGHT = 480, 480
N_PARTICLES = 5000
GRID_SIZE = 16
G = 1e-4
DT = 0.01

# Particle arrays (struct-of-arrays layout)
pos = np.zeros((N_PARTICLES, 2), dtype=np.float32)
vel = np.zeros((N_PARTICLES, 2), dtype=np.float32)
force = np.zeros((N_PARTICLES, 2), dtype=np.float32)
mass_grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
com_grid = np.zeros((GRID_SIZE, GRID_SIZE, 2), dtype=np.float32)

def init_particles():
    r = np.abs(np.random.normal(0.5, 0.25, N_PARTICLES))
    theta = np.random.uniform(0, 2 * np.pi, N_PARTICLES)
    pos[:, 0] = r * np.cos(theta)
    pos[:, 1] = r * np.sin(theta)
    v_mag = np.sqrt(G * N_PARTICLES / r) * np.random.normal(1., 0.05, N_PARTICLES)
    vel[:, 0] = -v_mag * np.sin(theta)
    vel[:, 1] = v_mag * np.cos(theta)
    force[:] = 0

def dynamics_update():
    global force, vel, pos, mass_grid, com_grid
    force.fill(0)
    r_norm = np.linalg.norm(pos, axis=1, keepdims=True) + 1e-2

    # Update mass grid
    bin_idx = np.clip(((pos + 1) * GRID_SIZE / 2).astype(np.int32), 0, GRID_SIZE - 1)
    mass_grid.fill(0)
    com_grid.fill(0)
    for xy, bin in zip(pos, bin_idx):
        mass_grid[bin[0], bin[1]] += 1
        com_grid[bin[0], bin[1], :] += xy

    # Start with center attraction
    #force[:] = -G * N_PARTICLES * pos / r_norm**3

    # And attract to each point in the mass grid
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            if mass_grid[i, j] > 1E-3:
                com = com_grid[i, j] / mass_grid[i, j]
                diff = com - pos
                dist = np.linalg.norm(diff, axis=1, keepdims=True)
                mask = (dist > 0.01).flatten()
                force[mask] += G * mass_grid[i, j] * diff[mask] / dist[mask]**3

    vel = vel + force * DT - 0.01 * np.clip(r_norm - 0.8, 0., np.inf) * pos / r_norm 
    pos = pos + vel * DT



def draw_particles():
    speeds = np.linalg.norm(vel, axis=1)
    vmax = np.percentile(speeds, 95) + 1e-5
    norm_speeds = np.clip(speeds / vmax, 0, 1)

    rgb = np.zeros((N_PARTICLES, 3), dtype=np.uint8)
    rgb[:, 0] = (255 * norm_speeds).astype(np.uint8)
    rgb[:, 1] = (128 * (1 - norm_speeds)).astype(np.uint8)
    rgb[:, 2] = (255 * (1 - norm_speeds)).astype(np.uint8)

    screen_xy = np.floor((pos + 1) * WIDTH // 2).astype(np.int32)
    img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

    mask = (screen_xy[:, 0] >= 0) & (screen_xy[:, 0] < WIDTH) & \
           (screen_xy[:, 1] >= 0) & (screen_xy[:, 1] < HEIGHT)
    img[screen_xy[mask, 1], screen_xy[mask, 0]] = rgb[mask]

    pygame.surfarray.blit_array(screen, img.swapaxes(0, 1))
    pygame.display.flip()



# Main loop
# Setup pygame
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

    clock.tick(120)

pygame.quit()
