import numpy as np

# Example numpy matrix
matrix = np.array([[1, 2, 3],
                   [4, 5, 6],
                   [7, 8, 9]])

# Iterate through all pairs of coordinates using ndindex
for index in np.ndindex(matrix.shape):
    print(f"Coordinate: {index}, Value: {matrix[index]}")
