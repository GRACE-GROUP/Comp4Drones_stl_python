import math
from typing import Union

import numpy as np


def rot_z(theta: float) -> np.ndarray:
    theta = math.radians(theta)
    rot = [[math.cos(theta), -math.sin(theta), 0],
           [math.sin(theta), math.cos(theta), 0],
           [0, 0, 1]]

    rot = np.array(rot)

    return rot
