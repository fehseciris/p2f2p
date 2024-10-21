import numpy as np
import matplotlib.pyplot as plt

# Read the waypoints and target from 'waypoints.txt'
waypoints = []
target_point = None

with open('waypoints.txt', 'r') as file:
    for line in file.readlines():
        parts = line.strip().split(',')
        if parts[0] == 'target':  # If the line contains the target
            target_point = (float(parts[1]), float(parts[2]))
        else:
            waypoints.append((float(parts[0]), float(parts[1])))

# Read the spline points from 'spline.txt'
spline_points = np.loadtxt('plot/spline.txt', delimiter=',')

# Separate X and Y coordinates for the waypoints and spline
waypoints_x = [point[0] for point in waypoints]
waypoints_y = [point[1] for point in waypoints]

spline_x = spline_points[:, 0]
spline_y = spline_points[:, 1]

# Plot the waypoints as blue circles
plt.scatter(waypoints_x, waypoints_y, color='blue', label='Waypoints')

# Plot the target point as a red 'X'
if target_point:
    plt.scatter(target_point[0], target_point[1], color='red', marker='x', s=100, label='Target')

# Plot the spline as a curve
plt.plot(spline_x, spline_y, color='green', label='Spline Curve')

# Add labels, grid, and legend
plt.title("Waypoints, Target, and Spline Curve")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.legend()

# Set axis limits based on the data range (optional, can be customized)
plt.xlim(min(min(waypoints_x), min(spline_x)) - 1, max(max(waypoints_x), max(spline_x)) + 1)
plt.ylim(min(min(waypoints_y), min(spline_y)) - 1, max(max(waypoints_y), max(spline_y)) + 1)

# Show the plot
plt.show()

# Eof 