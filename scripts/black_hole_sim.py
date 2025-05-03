import numpy as np
import pygame
import time
from collections import defaultdict
import matplotlib.pyplot as plt
import colorsys
import numba

# Screen size
WIDTH, HEIGHT = 480, 480
DRAW_DT = 1./30.

# Sim parameters
N_PARTICLES = 9000
# Particles have unit mass.
CENTER_MASS = 1000
G = 1e-5
TARGET_DT = 0.01
REAL_TIME_FACTOR = 1.0
PADDING = 1E-4
VELOCITY_DAMPING = 0.05
INITIAL_VELOCITY_PENALTY = 0.9
INITIAL_VELOCITY_VARIANCE = 0.1
MIN_DISTANCE = 0.02
MAX_SPEED = 0.5
EXPECTED_LIFTIME = 10
N_SPIRALS = 5

# State
ND = 3

bg_im = "assets/nebula.png"

# Pregenerate sprites of each radius. These are just gaussian kernels
MAX_RADIUS = 20
sprites = []
for radius in range(0, MAX_RADIUS+1):
    sprite = np.zeros((radius*2+1, radius*2+1))
    center_x = radius
    center_y = radius
    for u in range(radius):
        for v in range(radius):
            sqnorm = u*u+v*v
            falloff = np.exp(-sqnorm/radius)
            sprite[radius+u, radius+v] = falloff
            sprite[radius-u, radius+v] = falloff
            sprite[radius+u, radius-v] = falloff
            sprite[radius-u, radius-v] = falloff
    sprites.append(sprite)

BLACK_HOLE_RADIUS = 5
radius = BLACK_HOLE_RADIUS
black_hole_sprite = np.zeros((2*radius+1, 2*radius+1))
for u in range(radius):
    for v in range(radius):
        norm = np.sqrt(u * u + v * v)
        if norm <= radius:
            black_hole_sprite[radius+u, radius+v] = 1.0
            black_hole_sprite[radius-u, radius+v] = 1.0
            black_hole_sprite[radius+u, radius-v] = 1.0
            black_hole_sprite[radius-u, radius-v] = 1.0

class ParticleSim:
    def __init__(self):
        # State
        self.positions = np.zeros((ND, N_PARTICLES))
        self.velocities = np.zeros((ND, N_PARTICLES))

        self.last_dynamics_update = time.time()
        # Scratch space for dynamics update
        self.forces = np.zeros((ND, N_PARTICLES))
        self.colors = np.zeros((3, N_PARTICLES))

        # Start with random ages so they respawn at an even clip
        self.ages = EXPECTED_LIFTIME * np.arange(N_PARTICLES, dtype=float) / N_PARTICLES

        # Radiis and colors can be sampled once and persist across resets.
        hs = np.random.uniform(.6, .9, N_PARTICLES)
        ss = np.random.uniform(0.0, 0.4, N_PARTICLES)
        vs = np.random.uniform(0.2, 1.0, N_PARTICLES)
        self.colors = (np.stack([colorsys.hsv_to_rgb(h, s, v) for h, s, v in zip(hs, ss, vs)], axis=-1)*255).astype(np.uint8)
        self.radii = (np.random.geometric(p=0.2, size=N_PARTICLES).clip(1, MAX_RADIUS+1)-1).astype(np.uint32)

        self.set_normal_vector([0.5, 1.0, 0.85])

        # Current pixel coordinates (and scaled depth) of each star
        self.uvzs = np.zeros((3, N_PARTICLES), dtype=np.uint32)

        # Orthogonal projection
        self.K = np.zeros((3, ND))
        self.f = min(WIDTH, HEIGHT) / 2.0
        self.K[0, 0] = self.f
        self.K[1, 1] = self.f
        self.K[2, 2] = 1000.0
        self.center = np.array([WIDTH / 2.0, HEIGHT / 2.0, 0.0])


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
        ]).T

    @property
    def n_particles(self):
        return self.positions.shape[1]

    def initialize_all_particles(self):
        self.initialize_particles(np.full((N_PARTICLES,), True))
        # Start with random ages so they respawn at an even clip
        self.ages = EXPECTED_LIFTIME * np.arange(N_PARTICLES, dtype=float) / N_PARTICLES

    def initialize_particles(self, mask):
        theta = np.random.normal(0, 0.05, N_PARTICLES) + np.random.randint(0, N_SPIRALS, N_PARTICLES) * (2 * np.pi / N_SPIRALS)
        # Sample in coordinates of dominant plane of rotation
        r = np.abs(np.random.normal(0.75, 0.05, N_PARTICLES))
        u = r * np.cos(theta)
        v = r * np.sin(theta)
        uv = np.zeros((ND, N_PARTICLES)) 
        uv[0, :] = u
        uv[1, :] = v
        if ND >= 3:
            uv[2, :] = np.random.normal(0, 0.05, N_PARTICLES)
        self.positions[:, mask] = self.world_Q_planar @ uv[:, mask]
        
        speed = INITIAL_VELOCITY_PENALTY * np.sqrt(G * CENTER_MASS / r) * np.random.normal(1., INITIAL_VELOCITY_VARIANCE, N_PARTICLES)
        udot = -speed * np.sin(theta)
        vdot = speed * np.cos(theta)
        uv_dot = np.zeros((ND, N_PARTICLES))
        uv_dot[0, :] = udot
        uv_dot[1, :] = vdot
        if ND >= 3:
            uv_dot[2, :] = np.random.normal(0, 0.01, N_PARTICLES)
        self.velocities[:, mask] = self.world_Q_planar @ uv_dot[:, mask]
        self.ages[mask] = 0

    def dynamics_update(self, t):
        if t - self.last_dynamics_update < TARGET_DT:
            return
        dt = min(t - self.last_dynamics_update, TARGET_DT)
        self.last_dynamics_update = t
        self.ages += dt

        # Compute forces towards center.
        self.forces.fill(0)

        r_norm = np.linalg.norm(self.positions, axis=0) + PADDING
        # GMM / R^2   *  [pos / |pos|]
        self.forces = -G * CENTER_MASS * self.positions / r_norm**3 - self.velocities * VELOCITY_DAMPING
        self.velocities += self.forces * dt
        self.positions += self.velocities * dt

        # Respawn particles that have diverged.
        # too_far = np.linalg.norm(self.positions, axis=0) > 2.0
        # too_close = np.linalg.norm(self.positions, axis=0) < MIN_DISTANCE
        # too_fast = np.linalg.norm(self.velocities, axis=0) > MAX_SPEED
        # random = np.random.binomial(1, dt / EXPECTED_LIFTIME, N_PARTICLES) == 1
        too_old = self.ages >= EXPECTED_LIFTIME
        # print(f"too_far: {np.sum(too_far)}, too_close: {np.sum(too_close)}, too_fast: {np.sum(too_fast)}, random: {np.sum(random)}")
        self.initialize_particles(too_old)

    def view_update(self, t):
        # Update the view position.
        if ND == 3:
            view_direction = np.array([np.sin(0.2*t)*0.3, np.sin(0.3*t)*0.3, 0.9 + 0.1 * np.cos(0.3*t)])
            view_direction /= np.linalg.norm(view_direction)
            # Get two vectors orthogonal to the view direction.
            x_axis = np.array([1, 0, 0])
            if x_axis.dot(view_direction) > 0.99:
                # If the view direction is aligned with the x-axis, use y-axis instead.
                x_axis = np.array([0, 1, 0])
            y_axis = np.cross(view_direction, x_axis)
            x_axis = np.cross(y_axis, view_direction)
            z_axis = np.cross(x_axis, y_axis)
            self.K = np.array([
                x_axis * self.f,
                y_axis * self.f,
                z_axis * self.f,
            ]) 
        self.uvzs = self.K @ sim.positions
        self.uvzs = ((self.uvzs.T + self.center).T).astype(np.uint32)
        
@numba.jit()
def draw_particle(img, uvz, sprite, color, age, background_image, z_buffer):
    sprite_width = sprite.shape[0] 
    radius = int((sprite_width-1)/2)
    for dx in range(0, sprite_width):
        for dy in range(0, sprite_width):
            u = uvz[0] + dx - radius
            v = uvz[1] + dy - radius
            z = uvz[0]
            if u < 0 or u >= WIDTH or v < 0 or v >= HEIGHT:
                continue
            alpha = sprite[dx, dy]
            if (age < 1):
                alpha *= age
            elif (age >= EXPECTED_LIFTIME - 1):
                alpha *= (EXPECTED_LIFTIME - age)
            if z > z_buffer[u, v]:
                continue
            if alpha > 0.9:
                z_buffer[u, v] = z
            TRANSMISSIVITY = 0.5
            for k in range(3):
                new_value = int(img[u, v, k] * (1. - alpha*TRANSMISSIVITY) + alpha * color[k])
                img[u, v, k] = min(max(new_value, 0), 255)

@numba.jit()
def draw_background(img, uvz, radius, background_image):
    for dx in range(0, 2*radius+1):
        for dy in range(0, 2*radius+1):
            u = uvz[0] + dx - radius
            v = uvz[1] + dy - radius
            if u < 0 or u >= WIDTH or v < 0 or v >= HEIGHT:
                continue
            for k in range(3):
                img[u, v, k] = background_image[u, v, k]

class ParticleDrawing:
    ''''
        Illustrates particles using orthongonal projection. Camera is fixed
        looking at the origin, if in 3D, will rotate around it.
    '''
    def __init__(self, screen, sim):
        self.screen = screen
        self.sim = sim

        self.n_frames = 0
        self.last_draw = 0.0
        self.background_img = (plt.imread(bg_im)[:, :, :3]*255).astype(np.uint8)
        assert self.background_img.shape == (HEIGHT, WIDTH, 3)
        self.img = self.background_img.copy()
        self.z_buffer = np.zeros((HEIGHT, WIDTH))


    def update(self, t):
        if t - self.last_draw < DRAW_DT:
            return
        self.last_draw = t

        speeds = np.linalg.norm(sim.velocities, axis=0)
        
        fast_color = np.array([1.0, 0.7, 0.5])*255
        speed_ratio = np.clip(speeds / (MAX_SPEED * 0.75), 0, 1)**2.
        colors = speed_ratio[:, None] * fast_color + sim.colors.T * (1. - speed_ratio[:, None])

        #if ND == 3:
            # TODO: Darken ones that are "down" (-y)
            # down_dir = -sim.normal_vector
            # down_amount = down_dir.dot(sim.positions)
            # #max_down = np.max(down_amount)
            # #min_down = np.min(down_amount)
            # #down_amount = (down_amount - min_down) / (max_down - min_down)
            # colors *= down_amount[:, None]

        # Undraw previous stars
        for uvz, radius in zip(self.sim.uvzs.T, self.sim.radii):
            draw_background(self.img, uvz, radius, self.background_img)

        self.sim.view_update(t)

        # Draw new stars
        self.z_buffer[:] = 10000
        
        # Draw a black circle at the center.
        # draw_particle(self.img, np.array([WIDTH/2, HEIGHT/2, 0], dtype=np.uint32), black_hole_sprite, (0, 0, 0), EXPECTED_LIFTIME/2, self.background_img, self.z_buffer)

        for uvz, radius, color, age in zip(self.sim.uvzs.T, self.sim.radii, colors, self.sim.ages):
            draw_particle(self.img, uvz, sprites[radius], color, age, self.background_img, self.z_buffer)

        pygame.surfarray.blit_array(screen, self.img.swapaxes(0, 1))
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