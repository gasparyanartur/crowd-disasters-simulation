"""Functionality related to executing the simulation and generating a history."""
import numpy as np
from dataclasses import dataclass
from state import State


@dataclass(init=True, slots=True)
class SimConstants:
    time_inc: float = 0.01
    n_individuals: int = 100
    individual_radius: float = 1
    collision_rebound: float = 1
    mass: float = 10
    max_pos: tuple[int, int] = 50, 50
    n_time_steps: int = 10


def run_simulation(sim_constants: SimConstants, seed: int = None) -> list[State]:
    """Run the simulation step by step and generate a history.

    Args:
        n_time_steps: Number of time steps to run the simulation.
        seed: Gives the random number generator a reproducable result.

    Returns:
        The state for each time step.
    """
    np.random.seed(seed)

    max_pos = np.array(sim_constants.max_pos)
    zero_vectors = np.zeros(shape=(sim_constants.n_individuals, 2))
    positions = np.random.rand(sim_constants.n_individuals, 2)
    positions *= max_pos
    velocities = np.zeros(shape=(sim_constants.n_individuals, 2))

    state = State(positions=positions, velocities=velocities)
    history = [state]

    for time_step in range(sim_constants.n_time_steps):
        positions = state.positions + state.velocities*sim_constants.time_step
        forces = zero_vectors.copy()

        # TODO: Fix logic
        for individual in range(sim_constants.n_individuals):
            displacements = positions - positions[individual, :]
            distances = np.linalg.norm(displacements, axis=1)
            collisions = (distances < sim_constants.individual_radius) & (
                distances > 0)
            forces[collisions] += sim_constants.collision_rebound / \
                (displacements[collisions]**2)

        velocities = state.velocities + forces * \
            sim_constants.time_step / sim_constants.mass
        state = State(positions=positions, velocities=velocities)
        history.append(state)

    return history