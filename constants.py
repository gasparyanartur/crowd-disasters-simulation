import numpy as np


class Environment:    
    bot_outer = 0
    top_outer = 80
    left_outer = 0
    right_outer = 80
    
    thickness_top = 25
    thickness_bot = 25
    thickness_left = 10
    thickness_right = 10
    
    bot_inner = bot_outer + thickness_bot
    top_inner = top_outer - thickness_top
    left_inner = left_outer + thickness_left
    right_inner = right_outer - thickness_right
    
    
    # A rectangle is defined as (botleft, botright, topright, topleft)
    
    # Walls are defined as a set of points,
    # thus the total shape becomes (n_corners, n_dim)
    # example: rectangular obstacles -> shape=(4, 2)
    walls: list[np.ndarray] = [
        np.array([[left_outer, bot_outer], [right_outer, bot_outer], [right_outer, bot_inner], [left_outer, bot_inner]]),  # bot
        np.array([[right_inner, bot_inner], [right_outer, bot_inner], [right_outer, top_inner], [right_inner, top_inner]]),  # right
        np.array([[left_outer, top_inner], [right_outer, top_inner], [right_outer, top_outer], [left_outer, top_outer]]),  # top
        np.array([[left_outer, bot_inner], [left_inner, bot_inner], [left_inner, top_inner], [left_outer, top_inner]]),  # left
        
    ]
        
    wall_face: list[np.ndarray] = [
        np.array([0, 1]),
        np.array([-1, 0]),
        np.array([0, -1]),
        np.array([1, 0])
    ]
        
    # Exits are defined as rectangular,
    # and thus behave in the same way as walls
    exit_center = np.array([right_outer - left_outer, top_outer - bot_outer])
    exits: np.ndarray = np.array([
        [[right_inner-5, exit_center[1]-1], [right_outer, exit_center[1]-1], 
         [right_outer, exit_center[1]+1], [right_inner-5, exit_center[1]+1]]
    ])
        
    
class PersonState:
    living = 0
    exited = 1
    dead = 2
        
    
class SimConstants:
    time_inc = 0.01
    n_individuals = 1000
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

