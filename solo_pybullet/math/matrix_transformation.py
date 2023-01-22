import numpy as np


def transformation_matrix(rotation_matrix, translation_matrix):
    rotation_matrix = np.append(rotation_matrix, [[0, 0, 0]], 0)
    translation_matrix = np.append(translation_matrix, [1], 0)
    return np.column_stack([rotation_matrix, translation_matrix])
