import numpy as np

from sim_state import SimState
from constants import Environment as env
from constants import PersonState as pstate
from constants import SimConstants as sconst

def get_forces(state: SimState)  -> tuple[np.ndarray, np.ndarray]:
    soc_forces = np.zeros(shape=sconst.shape_2d)
    env_forces = np.zeros(shape=sconst.shape_2d)
    active = state.person_states == pstate.living
    
    exit = env.exits[0]
    exit_center = exit[0]+(exit[2]-exit[0])/2
    
    disps_exit =  exit_center - state.positions
    desired_dirs_exit = disps_exit / np.linalg.norm(disps_exit, axis=1)

    disps_wall = get_displacement_to_closest_wall(state)
    dist_wall = np.norm(disps_wall, axis=1)
    dirs_wall = disps_wall / dist_wall

    radius = sconst.individual_radius
    diameter = 2 * radius

    A = 1
    B = 1
    k1 = 100
    k2 = 100
    
    for i in range(sconst.n_individuals): 
        if not active[i]:
            continue
        
        velocity = state.velocities[i]
        position = state.positions[i]
        
        # Desire force
        desired_velocity = desired_dirs_exit[i]*sconst.v_max
        desired_force = sconst.individual_force * (desired_velocity - velocity)
        
        # Interaction force
        disps_ppl = position - state.positions
        vel_diff_ppl = state.velocities - velocity
        dist_ppl = np.norm(disps_ppl, axis=1)
        dirs_ppl = disps_ppl / dist_ppl 
        tang_ppl = np.array([-dirs_ppl[:, 1], dirs_ppl[:, 0]])
        closeness_ppl = diameter - dist_ppl
        tang_vel_ppl = vel_diff_ppl * tang_ppl

        i_collision_ppl = closeness_ppl <= 0
        
        interaction_force_soc = A * np.exp(closeness_ppl / B) * dirs_ppl
        interaction_force_env = i_collision_ppl * (k1 * dirs_ppl + k2 * tang_ppl * tang_vel_ppl)

        # Obstacle force
        # TODO: Obstacle force
        # dists_wall
        ...

    
    return env_forces, soc_forces


def get_displacement_to_closest_wall(state: SimState):
    x = state.positions[:, 0]
    y = state.positions[:, 1]

    w_bot_surf_y = env.walls[0][2, 1]
    w_right_surf_x = env.walls[1][0, 0]
    w_top_surf_y = env.walls[2][0, 1]
    w_left_surf_x = env.walls[3][1, 0]
    
    min_x = w_left_surf_x + sconst.distance_wall
    max_x = w_right_surf_x - sconst.distance_wall  
    min_y = w_bot_surf_y + sconst.distance_wall 
    max_y = w_top_surf_y - sconst.distance_wall 

    i_close_left = x < min_x
    i_close_right = x > max_x
    i_close_bot = y < min_y
    i_close_top = y > max_y
    
    disp = np.zeros((sconst.n_individuals, 2))

    disp[i_close_left, 0] = x - w_left_surf_x
    disp[i_close_right, 0] = x - w_right_surf_x
    disp[i_close_top, 1] = y - w_top_surf_y
    disp[i_close_bot, 1] = y + w_bot_surf_y

    return disp


def get_environmental_force(individual, state) -> float:
    """Net forces acting on an individual caused by the environment"""
    
    # Compute the displacement between this individual and all others, p_i - p_j.
    displacements = state.positions[individual, :] - state.positions

    # The distance to each individual is the norm of each displacement |p_i - p_j|.
    distances = np.linalg.norm(displacements, axis=1)

    # Find all the collisions that occur, which is defined as the instances
    # where the distances are within 2 radiuses, since this means that each
    # perimeter is just barely touching.
    # Exclude the instances where the distance is 0, since this means that
    # the individual is being compared to itself (or something has gone terribly wrong):
    collisions = (
        (state.person_states == pstate.living) &
        (distances < 2*sconst.individual_radius) &
        (distances > 0)
    )
    if collisions.size == 0:
        net_environmental_force = 0
    else:
        
        net_environmental_force = np.sum(sconst.collision_rebound / displacements[collisions], axis=0)
            
    return net_environmental_force
def get_obstacle_forces(state) -> np.ndarray:
    # The obstacle force between a given individual is defined as F_obs = e^(-d/r_0),
    # where d is the distance between the individual and the obstacle,
    # and r_0 is a parameter that determines how fast the force scaled with distance.
    
    obstacle_forces = np.zeros(shape=sconst.shape_2d)
    
    # TODO: Optimize
    for individual in range(sconst.n_individuals): 
        x = state.positions[individual,0]
        y = state.positions[individual,1]
        
        w_bot_surf_y = env.walls[0][2, 1]
        w_right_surf_x = env.walls[1][0, 0]
        w_top_surf_y = env.walls[2][0, 1]
        w_left_surf_x = env.walls[3][1, 0]
        
        disp = np.zeros((2,))
        
        if x < w_left_surf_x + sconst.distance_wall:
            disp[0] = w_left_surf_x - x
        
        elif x > w_right_surf_x - sconst.distance_wall :
            disp[0] = w_right_surf_x - x
            
        else:
            disp[0] = 0
            
            
        if y < w_bot_surf_y + sconst.distance_wall:
            disp[1] = w_bot_surf_y - y
            
        elif y > w_top_surf_y - sconst.distance_wall:
            disp[1] = w_top_surf_y - y
            
        else:
            disp[1] = 0
            
        dist = np.linalg.norm(disp)
        
        if dist == 0:
            force = 0
        else:
            force = disp / dist * math.exp(-dist/sconst.r_0)
        
        
        # TODO
        obstacle_forces[individual] = force


            
    return obstacle_forces
             
def get_environmental_forces(state):
    forces = np.zeros(shape=sconst.shape_2d)
    active = state.person_states == pstate.living
    
    for individual in range(sconst.n_individuals): 
        if not active[individual]:
            continue
                
        env_force = get_environmental_force(individual, state)
        forces[individual] = env_force
        
    for wall, wall_face in zip(env.walls, env.wall_face):
        i_colliding = get_touching_individuals_with_rectangle(state.positions, wall)
        if len(i_colliding[0]) == 0:
            continue

        # The wall will enact a force in the direction of the wallface.
        # The amount by which will be proportional to the projection of the 
        # velocity vector into the face (orthagonal) vector of the wall.
        #
        # Projecting the velocity vector into the face vector is done with the formula
        #   w = (v*u)/|u|^2 * u,
        # where v is the velocity vector and u is the face vector.
        # In this case, the face vector will always be normalised, so |u|^2 = 1
        #
        # Since the velocities are stored in a matrix, this can effeciently be done with
        #   W = V*u*u,
        # as this will essentially be reduced to
        #   w1 = v1*u*u, w2 = v2*u*u, ...
        # Observe that this is component-wise product, not dot product
        
        overlap = state.velocities[i_colliding]*wall_face
        forces[i_colliding] -= overlap * wall_face * sconst.wall_rebound
         
    return forces        
def get_desire_forces(state):
    forces = np.zeros(shape=sconst.shape_2d)
    active = state.person_states == pstate.living
    
    exit = env.exits[0]
    exit_center = exit[0]+(exit[2]-exit[0])/2
    
    for individual in range(sconst.n_individuals): 
        if not active[individual]:
            continue
            
        displacement = exit_center - state.positions[individual]
        desired_direction = displacement / np.linalg.norm(displacement)

        velocity = state.velocities[individual]    
        optimal_velocity = desired_direction*sconst.v_max

        desire_force = sconst.individual_force * (optimal_velocity - velocity)
            
        forces[individual] = desire_force
        
    return forces


def get_social_forces(state):
    
    forces = np.zeros(shape=sconst.shape_2d)
    active = state.person_states == pstate.living
    
    desire_forces = get_desire_forces(state)
    obstacle_forces = get_obstacle_forces(state)
    forces += desire_forces + obstacle_forces
    
    return forces
