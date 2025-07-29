# import matplotlib.pyplot as plt
# import matplotlib.patches as patches

# # Safe spots to draw
# safe_spots = [
#     (-3.8, 1.9),
#     (-0.6, 5.4),
#     (-3.5, 5.5)
# ]

# # Setup the plot
# plt.ion()  # Enable interactive mode
# fig, ax = plt.subplots()
# ax.set_xlim(-7.0, 5.0)
# ax.set_ylim(-2.0, 10.0)
# ax.set_title("Safe Spot Visualization")
# ax.set_xlabel("X")
# ax.set_ylabel("Y")
# ax.set_aspect('equal', adjustable='box')
# plt.grid(True)
# plt.show()  # Show empty plot window first

# # Pause briefly to ensure window opens
# plt.pause(0.1)

# # Wait for Enter in terminal
# input("✅ Plot window is open. Press Enter in the terminal to draw all safe spots...")

# # Draw all safe spots
# for x, y in safe_spots:
#     rect = patches.Rectangle((x - 0.5, y - 0.5), 2, 2,
#                              linewidth=1, edgecolor='green', facecolor='green', alpha=0.3)
#     ax.add_patch(rect)
#     ax.text(x, y, f'({x}, {y})', fontsize=6, ha='center', va='center', color='black')

# # Refresh the plot
# plt.draw()
# plt.pause(0.1)

# # Keep window open until closed manually
# plt.ioff()
# plt.show()

print("✅ mat.py is now executing")

while True:
    print("hello")