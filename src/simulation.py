"""Functionality related to executing the simulation and generating a history."""
import numpy as np
from dataclasses import dataclass
from state import State


@dataclass(slots=True)
class SimConstants:
    time_step = 0.01
    n_individuals = 100
    individual_radius = 1
    collision_rebound = 1
    mass = 10
    max_pos = np.array([50, 50])


def run_simulation(n_time_steps: int, seed: int = None) -> list[State]:
    np.random.seed(seed)

    zero_vectors = np.zeros(shape=(SimConstants.n_individuals, 2))
    positions = np.random.rand(SimConstants.n_individuals, 2)
    positions *= SimConstants.max_pos
    velocities = np.zeros(shape=(SimConstants.n_individuals, 2))

    state = State(positions=positions, velocities=velocities)
    history = [state]

    for time_step in range(n_time_steps):
        positions = state.positions + state.velocities*SimConstants.time_step
        forces = zero_vectors.copy()

        for individual in range(SimConstants.n_individuals):
            displacements = positions - positions[individual, :]
            distances = np.linalg.norm(displacements, axis=1)
            collisions = (distances < SimConstants.individual_radius) & (
                distances > 0)
            forces[collisions] += SimConstants.collision_rebound / \
                (displacements[collisions]**2)

        velocities = state.velocities + forces * \
            SimConstants.time_step / SimConstants.mass
        state = State(positions=positions, velocities=velocities)
        history.append(state)

    return history


def main():
    run_simulation()


if __name__ == "__main__":
    main()
