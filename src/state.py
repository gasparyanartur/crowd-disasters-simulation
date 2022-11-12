from dataclasses import dataclass
import numpy as np



@dataclass(init=True, slots=True)
class State:
    positions = np.ndarray
    velocities = np.ndarray

