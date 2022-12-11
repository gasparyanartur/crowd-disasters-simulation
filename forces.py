import numpy as np


def get_displacement_to_closest_wall(sconst, state, dim):
    if dim == 0:
        min_outer = sconst.left_inner
        max_outer = sconst.right_inner
    elif dim == 1:
        min_outer = sconst.bot_inner
        max_outer = sconst.top_inner

    min_inner = min_outer + sconst.distance_wall
    max_inner = max_outer - sconst.distance_wall

    p = state.positions[:, dim]
    i_close_min = p < min_inner
    i_close_max = p > max_inner

    disp = np.zeros((sconst.n_individuals, 2))
    disp[i_close_min, dim] = p[i_close_min] - min_outer
    disp[i_close_max, dim] = p[i_close_max] - max_outer

    return disp


def get_obstacle_forces(sconst, state, dim):
    disps_wall = get_displacement_to_closest_wall(sconst, state, dim)
    dist_wall = np.linalg.norm(disps_wall, axis=1, keepdims=True)
    nonzero_dist_wall = (dist_wall > 0).flatten()
    dirs_wall = np.zeros(shape=disps_wall.shape)
    dirs_wall[nonzero_dist_wall] = (
        disps_wall[nonzero_dist_wall] / dist_wall[nonzero_dist_wall])
    tang_wall = np.vstack(
        (-dirs_wall[:, 1], dirs_wall[:, 0])).T        # normal vector
    closeness_wall = sconst.individual_radius - dist_wall
    tang_vel_wall = state.velocities * tang_wall
    i_collision_wall = closeness_wall >= 0

    obstacle_forces_soc = sconst.A * np.exp(closeness_wall / sconst.B) * dirs_wall
    obstacle_forces_env = i_collision_wall * (sconst.k1 * dirs_wall - sconst.k2 * tang_vel_wall * tang_wall)

    return obstacle_forces_env, obstacle_forces_soc


def get_forces(sconst, state) -> tuple[np.ndarray, np.ndarray]:
    soc_forces = np.zeros(shape=sconst.shape_2d)
    env_forces = np.zeros(shape=sconst.shape_2d)
    active = state.person_states == sconst.s_living

    exit = sconst.exits[0]
    exit_center = exit[0]+(exit[2]-exit[0])/2

    disps_exit = exit_center - state.positions
    desired_dirs_exit = disps_exit / np.linalg.norm(disps_exit, axis=1, keepdims=True)

    # Obstacle force
    obstacle_forces_soc_x, obstacle_forces_env_x = get_obstacle_forces(sconst, state, 0)
    obstacle_forces_soc_y, obstacle_forces_env_y = get_obstacle_forces(sconst, state, 1)

    # Desire force
    desired_velocity = desired_dirs_exit*sconst.v_max
    desired_forces = sconst.individual_force * (desired_velocity - state.velocities)

    
    for i in range(sconst.n_individuals):
        if not active[i]:
            continue

        velocity = state.velocities[i]
        position = state.positions[i]

        # Interaction force
        disps_ppl = position - state.positions[active]
        vel_diff_ppl = state.velocities[active] - velocity
        dist_ppl = np.linalg.norm(disps_ppl, axis=1, keepdims=True)
        dist_ppl[dist_ppl == 0] = 1
        dirs_ppl = disps_ppl / dist_ppl
        tang_ppl = np.flip(dirs_ppl, axis=1).copy()
        tang_ppl[:, 0] *= -1
        closeness_ppl = 2*sconst.individual_radius - dist_ppl
        tang_vel_ppl = vel_diff_ppl * tang_ppl
        interaction_force_soc = sconst.A * np.exp(closeness_ppl / sconst.B) * dirs_ppl

        i_collision_ppl = closeness_ppl >= 0
        interaction_force_env = i_collision_ppl * (sconst.k1 * dirs_ppl + sconst.k2 * tang_ppl * tang_vel_ppl)

        env_forces[i] += np.sum(interaction_force_env, axis=0)
        soc_forces[i] += np.sum(interaction_force_soc, axis=0)

    soc_forces[active] += (desired_forces[active] +
                           obstacle_forces_soc_x[active] + obstacle_forces_soc_y[active])

    env_forces[active] += (obstacle_forces_env_x[active] +
                           obstacle_forces_env_y[active])

    return env_forces, soc_forces
