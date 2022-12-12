import numpy as np
import pandas as pd

from dataclasses import dataclass


def history_to_dataframe(sconst, history):
    frames = []

    for i in range(sconst.n_time_steps):
        frame = history[i]
        stacked_frame = np.hstack([
            frame.positions,
            frame.velocities,
            frame.forces,
            frame.pressures.reshape(-1, 1),
            frame.person_states.reshape(-1, 1)
        ])
        frames.append(stacked_frame)

    stacked_history = np.vstack(frames)
    stacked_history.shape
    return pd.DataFrame(stacked_history, columns=[
        "position_x", "position_y",
        "velocity_x", "velocity_y",
        "force_x", "force_y",
        "pressure", "person_state"
    ])


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
            positions=self.positions.copy(),
            velocities=self.velocities.copy(),
            forces=self.forces.copy(),
            pressures=self.pressures.copy(),
            person_states=self.person_states.copy()
        )
