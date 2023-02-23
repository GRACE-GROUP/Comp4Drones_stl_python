import math

import numpy as np


def nchoosek(n, k):
    return math.comb(n, k)


def nchoosek_vector_2(start: int, end: int) -> np.ndarray:
    res = []
    for i in range(start, end):
        for j in range(i + 1, end + 1):
            res.append([i, j])
    res = np.array(res)
    return res

