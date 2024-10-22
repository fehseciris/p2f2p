import numpy as np
import matplotlib.pyplot as plt
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
os.chdir(script_dir)

# Load the waypoints and handle target identification
waypoints = []
target_point = None

# Open and read the waypoints file
with open('waypoints.txt', 'r') as file:
    for line in file.readlines():
        parts = line.strip().split(',')
        if parts[0].lower() == 'target':
            target_point = (float(parts[1]), float(parts[2]))
        else:
            waypoints.append((float(parts[0]), float(parts[1])))

# Separate the waypoints into x and y coordinates
waypoints_x = [point[0] for point in waypoints]
waypoints_y = [point[1] for point in waypoints]

# Define the polynomial function
def polynomial(x, a0, a1, a2, a3):
    return a3 * x**3 + a2 * x**2 + a1 * x + a0

# Load the coefficients from the file
coeffs = np.loadtxt("coefficients.txt", delimiter=",")

# Define the x-values for plotting (extended range from -2 to 10)
x_vals = np.linspace(-2, 10, 1000)

# Plot each polynomial
plt.figure(figsize=(10, 6))
for idx, (a0, a1, a2, a3) in enumerate(coeffs):
    y_vals = polynomial(x_vals, a0, a1, a2, a3)
    plt.plot(x_vals, y_vals, label=f"Polynomial {idx + 1}")

# Plot the waypoints as blue circles
plt.scatter(waypoints_x, waypoints_y, color='blue', label="Waypoints", zorder=5)

# Plot the target point as a red 'X', if present
if target_point:
    target_x, target_y = target_point
    plt.scatter(target_x, target_y, color='red', marker='x', s=100, label="Target", zorder=6)

# Add labels, grid, and legend
plt.title("Fitted Polynomials with Waypoints and Target")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.legend()

# Set axis limits to [-2, 10]
plt.xlim([-2, 10])
plt.ylim([-2, 10])

# Show the plot
plt.show()

# Eof