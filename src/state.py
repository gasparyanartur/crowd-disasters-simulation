from dataclasses import dataclass
import numpy as np



@dataclass
class State(init=True, slots=True):
    positions = np.ndarray
    velocities = np.ndarray

