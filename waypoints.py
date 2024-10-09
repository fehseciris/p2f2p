import matplotlib.pyplot as plt

# Datei mit Wegpunkten öffnen und lesen
waypoints = []
target_point = None

with open('waypoints.txt', 'r') as file:
    for line in file.readlines():
        parts = line.strip().split(',')
        if parts[0] == 'target':
            target_point = (float(parts[1]), float(parts[2]))
        else:
            waypoints.append((float(parts[0]), float(parts[1])))

# X und Y Koordinaten trennen
x_coords = [point[0] for point in waypoints]
y_coords = [point[1] for point in waypoints]

# Plot erstellen
plt.figure(figsize=(8, 6))
plt.plot(x_coords, y_coords, 'bo-', label="Waypoints")  # Original waypoints plot

# Den Zielpunkt separat einzeichnen
if target_point:
    plt.plot(target_point[0], target_point[1], 'ro', label="Target Point", markersize=10)  # Rot markierter Punkt

# Achsenbeschriftung und Titel
plt.title("Waypoint Coordinates with Target Point")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.legend()

# Plot anzeigen
plt.show()

# Eof