import pygame
import time
import random
import math
import numpy as np

# Parameters
WIDTH, HEIGHT = 480, 480
N_PARTICLES = 10000
GRID_SIZE = 16
G = 1E-4
DT = 0.01
CELL_WIDTH = 2 / GRID_SIZE  # normalized units

# Setup pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# Particle structure
class Particle:
    def __init__(self, x, y, vx, vy):
        self.pos = np.array([x, y], dtype=np.float32)
        self.vel = np.array([vx, vy], dtype=np.float32)
        self.force = np.zeros(2, dtype=np.float32)

particles = []

def init_particles():
    particles.clear()
    for _ in range(N_PARTICLES):
        r = random.uniform(0.2, 0.6)
        theta = random.uniform(0, 2 * math.pi)
        x, y = r * math.cos(theta), r * math.sin(theta)
        v_mag = 0.5 * math.sqrt(G * N_PARTICLES / r)
        vx = -v_mag * math.sin(theta)
        vy = v_mag * math.cos(theta)
        particles.append(Particle(x, y, vx, vy))


def to_screen(p):
    return int((p[0] + 1) * WIDTH // 2), int((1 - p[1]) * HEIGHT // 2)

def grid_index(p):
    ix = int((p[0] + 1) / 2 * GRID_SIZE)
    iy = int((p[1] + 1) / 2 * GRID_SIZE)
    return max(0, min(GRID_SIZE - 1, ix)), max(0, min(GRID_SIZE - 1, iy))

def compute_density_grid(particles):
    mass_grid = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
    com_grid = np.zeros((GRID_SIZE, GRID_SIZE, 2), dtype=np.float32)

    for p in particles:
        ix, iy = grid_index(p.pos)
        mass_grid[ix, iy] += 1.0  # unit mass
        com_grid[ix, iy] += p.pos

    # Avoid divide-by-zero
    mask = mass_grid > 0
    com_grid[mask] /= mass_grid[mask][:, None]
    return mass_grid, com_grid

def compute_forces_from_grid(particles, mass_grid, com_grid):
    # Flatten the grid into (num_cells, 2) and (num_cells,) arrays
    grid_centers = com_grid.reshape(-1, 2)
    grid_masses = mass_grid.flatten()

    # Filter out empty grid cells
    valid = grid_masses > 0
    mass_sources = grid_masses[valid]
    center_sources = grid_centers[valid]

    # Stack all particle positions into an array: (N, 2)
    particle_pos = np.array([p.pos for p in particles])
    forces = np.zeros_like(particle_pos)

    for i, center in enumerate(center_sources):
        delta = center - particle_pos  # shape: (N, 2)
        dist2 = np.sum(delta**2, axis=1) + 1e-3
        force_mag = G * mass_sources[i] / dist2
        force_vecs = delta * (force_mag / np.sqrt(dist2))[:, np.newaxis]
        forces += force_vecs

    # Apply the forces back to particles plus some bulk cheat forces to influence bulk motion.
    for i, p in enumerate(particles):
        p.force[:] = 0.1 * forces[i]

        # Direct motion damping
        p.force -= 0. * p.vel

        # Weak inwards force
        r = p.pos
        r_norm = np.linalg.norm(r) + 0.1
        inward_force = -G * r * N_PARTICLES / (r_norm**3)
        p.force += inward_force

        # Add a weak tangential force to induce rotation
        tangent = np.array([-r[1], r[0]])
        tangent /= r_norm  # unit perpendicular to radius
        target_speed = 0.01
        actual_speed = np.dot(p.vel, tangent)
        correction = target_speed - actual_speed
        p.force += correction * tangent * 0.02  # PD controller style



def update_particles(particles):
    for p in particles:
        p.vel += p.force * DT
        p.pos += p.vel * DT

def draw_particles(screen, particles):
    screen.fill((0, 0, 0))

    # Estimate max velocity for normalization
    speeds = np.array([np.linalg.norm(p.vel) for p in particles])
    vmax = np.percentile(speeds, 95) + 1e-5  # ignore outliers

    for p in particles:
        x, y = to_screen(p.pos)
        if 0 <= x < WIDTH and 0 <= y < HEIGHT:
            speed = np.linalg.norm(p.vel)
            norm_speed = min(speed / vmax, 1.0)
            r = int(255 * norm_speed)
            g = int(128 * (1 - norm_speed))
            b = int(255 * (1 - norm_speed))

            screen.set_at((x, y), (r, g, b))

    pygame.display.flip()


# Main loop with profiling
running = True
init_particles()
while running:
    frame_start = time.time()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_BACKSPACE:
                init_particles()

    t0 = time.time()
    mass_grid, com_grid = compute_density_grid(particles)
    t1 = time.time()
    compute_forces_from_grid(particles, mass_grid, com_grid)
    t2 = time.time()
    update_particles(particles)
    t3 = time.time()
    draw_particles(screen, particles)
    t4 = time.time()

    print(f"Grid: {(t1 - t0)*1000:.1f} ms | Force: {(t2 - t1)*1000:.1f} ms | "
          f"Update: {(t3 - t2)*1000:.1f} ms | Draw: {(t4 - t3)*1000:.1f} ms | "
          f"Total: {(t4 - frame_start)*1000:.1f} ms")

    clock.tick(60)


pygame.quit()
