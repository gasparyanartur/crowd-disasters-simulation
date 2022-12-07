import numpy

from sim_state import SimState
from constants import Environment as env
from constants import PersonState as pstate
from constants import SimConstants as sconst

def get_forces(state: State)  -> tuple[np.ndarray, np.ndarray]:
    soc_forces = np.zeros(shape=sconst.shape_2d)
    env_forces = np.zeros(shape=sconst.shape_2d)
    active = state.person_states == pstate.living
    
    exit = env.exits[0]
    exit_center = exit[0]+(exit[2]-exit[0])/2
    
    disps_exit =  exit_center - state.positions
    desired_dirs_exit = disps_exit / np.linalg.norm(disps_exit, axis=1)
    
    for individual in range(sconst.n_individuals): 
        if not active[individual]:
            continue
        
        velocity = state.velocities[individual]    
        desired_velocity = desired_dirs_exit[individual]*sconst.v_max
        desired_force = sconst.individual_force * (desired_velocity - velocity)
            
        
        

    
    
    
   
    
    
    
    
    return env_forces, soc_forces


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
