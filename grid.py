import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Grid parameters
grid_width = 12     # meters
grid_height = 9     # meters
cell_size = 0.5     # each cell is 0.5 m

# Create figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Draw grid lines along X and Y axes on Z=0 plane
x_lines = int(grid_width / cell_size) + 1
y_lines = int(grid_height / cell_size) + 1

# Vertical lines (along Y)
for i in range(x_lines):
    x = i * cell_size
    ax.plot([x, x], [0, grid_height], [0, 0], color='gray', linewidth=0.5)

# Horizontal lines (along X)
for j in range(y_lines):
    y = j * cell_size
    ax.plot([0, grid_width], [y, y], [0, 0], color='gray', linewidth=0.5)

# Set axes properties
ax.set_xlim(0, grid_width)
ax.set_ylim(0, grid_height)
ax.set_zlim(0, 1)  # a small height just to allow 3D perspective

ax.set_xlabel("X (meters)")
ax.set_ylabel("Y (meters)")
ax.set_zlabel("Z (meters)")

# Set title
ax.set_title("Safe Spots")

plt.tight_layout()
plt.show()