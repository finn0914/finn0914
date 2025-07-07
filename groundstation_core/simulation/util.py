import numpy as np


# Takes tuples instead of numpy arrays.
# Returns `numpy.float64`.
def euclidean_distance(pos1, pos2):
    return np.linalg.norm(np.array(pos1) - np.array(pos2))
