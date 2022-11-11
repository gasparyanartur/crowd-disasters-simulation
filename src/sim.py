import numpy as np


class Constants:
    n_individuals = 100
    individual_radius = 1
    time_step = 0.01
    collision_rebound = 1
    mass = 10
    max_x, max_y = 50, 50


def run_simulation():
    np.random.seed(69420)

    zero_vectors = np.zeros(shape=(Constants.n_individuals, 2))
    positions = np.random.rand(Constants.n_individuals, 2)
    positions[:, 0] *= Constants.max_y
    positions[:, 1] *= Constants.max_x
    velocities = np.zeros(shape=(Constants.n_individuals, 2)) 
    
    history = [(positions, velocities)]

    for t in range(3):
        old_positions, old_velocities = history[-1]
        positions = old_positions + old_velocities*Constants.time_step
        forces = zero_vectors.copy()

        for individual in range(Constants.n_individuals):
            displacements = positions - positions[individual, :]
            distances = np.linalg.norm(displacements, axis=1) 
            collisions = (distances < Constants.individual_radius) & (distances > 0)
            forces[collisions] += Constants.collision_rebound/(displacements[collisions]**2)

        velocities = old_velocities + forces*Constants.time_step / Constants.mass
        history.append((positions, velocities))
        

    print(history[-1])



def main():
    run_simulation()

if __name__ == "__main__":
    main()