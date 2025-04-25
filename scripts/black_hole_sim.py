import numpy as np
import pygame
import time
from collections import defaultdict
import matplotlib.pyplot as plt

# Screen size
WIDTH, HEIGHT = 480, 480
DRAW_DT = 1./30.

# Sim parameters
N_PARTICLES = 50000
# Particles have unit mass.
CENTER_MASS = 1000
G = 1e-4
TARGET_DT = 0.01
REAL_TIME_FACTOR = 1.0
PADDING = 1E-2

# State
ND = 3

class ParticleSim:
    def __init__(self):
        # State
        self.positions = np.zeros((ND, N_PARTICLES))
        self.velocities = np.zeros((ND, N_PARTICLES))

        self.last_dynamics_update = time.time()
        # Scratch space for dynamics update
        self.forces = np.zeros((ND, N_PARTICLES))
        self.colors = np.zeros((3, N_PARTICLES))

        self.set_normal_vector([1.0, 1.0, 1.0])

    def set_normal_vector(self, normal_vector):
        self.normal_vector = normal_vector / np.linalg.norm(normal_vector)

        # Particles will rotate about the normal vector.
        x_axis = np.array([1, 0, 0])
        if x_axis.dot(self.normal_vector) > 0.99:
            # If the normal vector is aligned with the x-axis, use y-axis instead.
            x_axis = np.array([0, 1, 0])
        y_axis = np.cross(self.normal_vector, x_axis)
        x_axis = np.cross(y_axis, self.normal_vector)
        self.world_Q_planar = np.array([
            x_axis,
            y_axis,
            self.normal_vector
        ])

    @property
    def n_particles(self):
        return self.positions.shape[1]

    def initialize_all_particles(self):
        self.initialize_particles(np.full((N_PARTICLES,), True))

    def initialize_particles(self, mask):
        N_SPIRALS = 8
        theta = np.random.normal(0, 0.1, N_PARTICLES) + np.random.randint(0, N_SPIRALS, N_PARTICLES) * (2 * np.pi / N_SPIRALS)
        # Sample in coordinates of dominant plane of rotation
        r = np.abs(np.random.normal(0.75, 0.1, N_PARTICLES))
        u = r * np.cos(theta)
        v = r * np.sin(theta)
        uv = np.zeros((ND, N_PARTICLES)) 
        uv[0, :] = u
        uv[1, :] = v
        if ND >= 3:
            uv[2, :] = np.random.normal(0, 0.01, N_PARTICLES)
        self.positions[:, mask] = self.world_Q_planar @ uv[:, mask]
        
        speed = np.sqrt(G * CENTER_MASS / r) * np.random.normal(1., 0.05, N_PARTICLES)
        udot = -speed * np.sin(theta)
        vdot = speed * np.cos(theta)
        uv_dot = np.zeros((ND, N_PARTICLES))
        uv_dot[0, :] = udot
        uv_dot[1, :] = vdot
        if ND >= 3:
            uv_dot[2, :] = np.random.normal(0, 0.01, N_PARTICLES)
        self.velocities[:, mask] = self.world_Q_planar @ uv_dot[:, mask]

        self.colors[:, mask] = plt.get_cmap("Blues")(np.random.uniform(0.0, 1.0, N_PARTICLES))[mask, :3].T


    def dynamics_update(self, t):
        if t - self.last_dynamics_update < TARGET_DT:
            return
        dt = t - self.last_dynamics_update
        self.last_dynamics_update = t

        # Compute forces towards center.
        self.forces.fill(0)

        r_norm = np.linalg.norm(self.positions, axis=0) + PADDING
        # GMM / R^2   *  [pos / |pos|]
        self.forces = -G * CENTER_MASS * self.positions / r_norm**3
        self.velocities += self.forces * dt
        self.positions += self.velocities * dt

        # Respawn particles that have diverged.
        too_far = np.linalg.norm(self.positions, axis=0) > 1.0
        too_close = np.linalg.norm(self.positions, axis=0) < 0.1
        too_fast = np.linalg.norm(self.velocities, axis=0) > 1.0
        random = np.random.binomial(1, 0.01, N_PARTICLES)
        self.initialize_particles(too_far | too_close | too_fast | random)


class ParticleDrawing:
    ''''
        Illustrates particles using orthongonal projection. Camera is fixed
        looking at the origin, if in 3D, will rotate around it.
    '''
    def __init__(self, screen, sim):
        self.screen = screen
        self.sim = sim

        self.n_frames = 0

        self.K = np.zeros((2, ND))
        self.f = min(WIDTH, HEIGHT) / 2.0
        self.K[0, 0] = self.f
        self.K[1, 1] = self.f
        self.center = np.array([WIDTH / 2.0, HEIGHT / 2.0])

        self.last_draw = 0.0

    def update(self, t):
        if t - self.last_draw < DRAW_DT:
            return
        self.last_draw = t

        # Update the view position.
        if ND == 3:
            view_direction = np.array([np.sin(0.1*t)*0.1, np.sin(0.2*t)*0.1, 0.9 + 0.1 * np.cos(0.3*t)])
            view_direction /= np.linalg.norm(view_direction)
            # Get two vectors orthogonal to the view direction.
            x_axis = np.array([1, 0, 0])
            if x_axis.dot(view_direction) > 0.99:
                # If the view direction is aligned with the x-axis, use y-axis instead.
                x_axis = np.array([0, 1, 0])
            y_axis = np.cross(view_direction, x_axis)
            x_axis = np.cross(y_axis, view_direction)
            self.K = np.array([
                x_axis,
                y_axis,
            ]) * self.f



        self.screen.fill((0, 0, 0))
        
        # Draw the orbiting stars, with color based on speed with some unique per-star jitter.
        uv = self.K @ sim.positions
        uv = ((uv.T + self.center).T).astype(np.int32)
        
        speeds = np.linalg.norm(sim.velocities, axis=0)
        max_speed = 0.7

        fast_color = np.array([1.0, 0.4, 0.1])
        speed_ratio = np.clip(speeds / max_speed, 0, 1)**2.
        colors = speed_ratio[:, None] * fast_color + sim.colors.T * (1. - speed_ratio[:, None])
            
        img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        mask = (uv[0, :] >= 0) & (uv[0, :] < WIDTH) & \
            (uv[1, :] >= 0) & (uv[1, :] < HEIGHT)
        img[uv[1, mask], uv[0, mask]] = colors[mask, :]*255

        pygame.surfarray.blit_array(screen, img.swapaxes(0, 1))
        pygame.display.flip()
        self.n_frames += 1


if __name__ == "__main__":
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Particle Simulation")


    sim = ParticleSim()
    vis = ParticleDrawing(sim=sim, screen=screen)
    
    # Init display
    running = True

    sim.initialize_all_particles()
    
    # Main loop
    cumulative_times = defaultdict(float)
    start_time = time.time()
    last_print = time.time()

    while running:
        t = time.time()

        times = []
        names = []

        # Get time since last frame
        times.append(time.time())
        names.append("start")

        # Update dynamics
        sim.dynamics_update(t)

        times.append(time.time())
        names.append("dynamics_update")

        # Draw particles
        vis.update(t)

        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_BACKSPACE:
                sim.initialize_all_particles()
        times.append(time.time())
        names.append("draw")

        for i in range(len(times) - 1):
            name = f"{names[i]}_to_{names[i + 1]}"
            dt = times[i + 1] - times[i]
            cumulative_times[name] += dt

        if time.time() - last_print > 1.0:
            last_print = time.time()
            framerate = vis.n_frames / (time.time() - start_time)
            total_cumulative_times = sum(cumulative_times.values())
            print(f"FPS {framerate:.1f} | " + " | ".join([f"{name}: {total_t / total_cumulative_times*100:0.1f}%" for name, total_t in cumulative_times.items()]))