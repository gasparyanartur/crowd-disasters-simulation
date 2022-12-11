from dataclasses import dataclass
import numpy as np

class SimConstants:
    def __init__(self, *, time_inc=0.01, n_individuals=500, individual_radius=0.5,
                          simulation_time=10, v_max=5, distance_wall=4, mass=100,
                          individual_force=150, lethal_pressure=30000,
                          A=2000, B=0.08, k1=1200, k2=2400,
                          width=20, length=60, exit_width=2, exit_length=3):

        self.time_inc = time_inc
        self.n_individuals = n_individuals
        self.individual_radius = individual_radius
        self.mass = mass
        self.simulation_time = simulation_time
        self.v_max = v_max
        self.individual_force = individual_force
        self.lethal_pressure = lethal_pressure
        self.distance_wall = distance_wall
        
        self.A = A
        self.B = B
        self.k1 = k1
        self.k2 = k2

        self.n_time_steps = int(self.simulation_time/self.time_inc)
        self.force_scalar = self.time_inc/self.mass
        self.shape_2d = (self.n_individuals, 2)
        self.shape_1d = (self.n_individuals, )

        self.width = width
        self.length = length

        self.exit_width = exit_width
        self.exit_length = exit_length

        self.bot_outer = 0
        self.top_outer = 80
        self.left_outer = 0
        self.right_outer = 80

        self.thickness_bot = self.thickness_top = (self.top_outer-self.bot_outer-width)//2
        self.thickness_left = self.thickness_right = (self.right_outer-self.left_outer-length)//2

        self.bot_inner = self.bot_outer + self.thickness_bot
        self.top_inner = self.top_outer - self.thickness_top
        self.left_inner = self.left_outer + self.thickness_left
        self.right_inner = self.right_outer - self.thickness_right

        # A rectangle is defined as (botleft, botright, topright, topleft)

        # Walls are defined as a set of points,
        # thus the total shape becomes (n_corners, n_dim)
        # example: rectangular obstacles -> shape=(4, 2)
        self.walls: list[np.ndarray] = [
            np.array([
                [self.left_outer, self.bot_outer], [self.right_outer, self.bot_outer], 
                [self.right_outer, self.bot_inner], [self.left_outer, self.bot_inner]
            ]),  # bot
            np.array([
                [self.right_inner, self.bot_inner], [self.right_outer, self.bot_inner],
                [self.right_outer, self.top_inner], [self.right_inner, self.top_inner]
            ]),  # right
            np.array([
                [self.left_outer, self.top_inner], [self.right_outer, self.top_inner], 
                [self.right_outer, self.top_outer], [self.left_outer, self.top_outer]
            ]),  # top
            np.array([
                [self.left_outer, self.bot_inner], [self.left_inner, self.bot_inner], 
                [self.left_inner, self.top_inner], [self.left_outer, self.top_inner]
            ]),  # left
        ]

        self.center = np.array([
            self.left_inner+(self.right_inner - self.left_inner)/2,
            self.bot_inner+(self.top_inner - self.bot_inner)/2
        ])

        self.exits: np.ndarray = np.array([[
            [self.right_inner-exit_length//2, self.center[1]-exit_width//2],
            [self.right_inner+exit_length//2, self.center[1]-exit_width//2], 
            [self.right_inner+exit_length//2, self.center[1]+exit_width//2],
            [self.right_inner-exit_length//2, self.center[1]+exit_width//2]
        ]])

        self.s_living = 0
        self.s_exited = 1
        self.s_dead = 2