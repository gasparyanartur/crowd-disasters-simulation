import numpy as np


class Environment:    
    bot_outer = 0
    top_outer = 50
    left_outer = 0
    right_outer = 50
    
    wall_thickness = 2
    bot_inner = bot_outer + wall_thickness
    top_inner = top_outer - wall_thickness
    left_inner = left_outer + wall_thickness
    right_inner = right_outer - wall_thickness
    
    
    # A rectangle is defined as (botleft, botright, topright, topleft)
    
    # Walls are defined as a set of points,
    # thus the total shape becomes (n_corners, n_dim)
    # example: rectangular obstacles -> shape=(4, 2)
    walls: list[np.ndarray] = [
        np.array([[0, 0], [50, 0], [50, 2], [0, 2]]),  # bot
        np.array([[48, 2], [50, 2], [50, 48], [48, 48]]),  # right
        np.array([[0, 48], [50, 48], [50, 50], [0, 50]]),  # top
        np.array([[0, 2], [2, 2], [2, 48], [0, 48]]),  # left
        
    ]
        
    wall_face: list[np.ndarray] = [
        np.array([0, 1]),
        np.array([-1, 0]),
        np.array([0, -1]),
        np.array([1, 0])
    ]
        
    # Exits are defined as rectangular,
    # and thus behave in the same way as walls
    exits: np.ndarray = np.array([
        [[40, 24], [51, 24], [51, 26], [40, 26]]
    ])
        
    
class PersonState:
    living = 0
    exited = 1
    dead = 2
        
    
class SimConstants:
    time_inc = 0.01
    n_individuals = 200
    individual_radius = 0.5
    collision_rebound = 100
    wall_rebound = 1000
    mass = 100
    max_pos = 50, 50
    simulation_time = 10
    damping_constant = 0
    start_margin = 0.1
    social_factor = 50
    v_max = 5
    individual_force = 100
    lethal_pressure = 15000
    distance_wall = 4
    r_0 = 1 # the constant for calculate the obstacle force
    n_time_steps = int(simulation_time/time_inc)
    force_scalar = time_inc/mass
    shape_2d = (n_individuals, 2)
    shape_1d = (n_individuals, )

