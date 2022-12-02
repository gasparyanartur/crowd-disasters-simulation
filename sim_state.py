from dataclasses import dataclass


@dataclass(init=True)
class SimState:
    def __init__(self, positions, velocities, forces, pressures, person_states): 
        self.positions: np.ndarray = positions
        self.velocities: np.ndarray = velocities
        self.forces: np.ndarray = forces
        self.pressures: np.ndarray = pressures
        self.person_states: np.ndarray = person_states
            
        
    def copy(self):
        return SimState(
            positions = self.positions.copy(),
            velocities = self.velocities.copy(),
            forces = self.forces.copy(),
            pressures = self.pressures.copy(),
            person_states = self.person_states.copy()
        )
