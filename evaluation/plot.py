import matplotlib.pyplot as plt
import numpy as np

data = []
with open("sensor_fusion-9-stdout.log.a") as f:
	data = f.read()
data = data.split("\n")
data = filter(None, data)

lines = [[], [], [], [], [], [], []]
for i in data:
	i = i.split(" ")
	lines[0].append(float(i[0]))
	lines[1].append(float(i[1]))
	lines[2].append(float(i[2]))
	lines[3].append(float(i[3]))
	lines[4].append(float(i[4]))
	lines[5].append(float(i[5]))
	lines[6].append(float(i[6]))
t = np.arange(0.0, len(lines[0]), 1)
plt.plot(t, lines[0], label="Lidar")
plt.plot(t, lines[1], label="SVM")
plt.plot(t, lines[2], label="Our framework")
plt.plot(t, lines[3], label="Kalman Filter SVM")
plt.plot(t, lines[4], label="Switching Filter SVM")
plt.plot(t, lines[4], label="Switching Filter PointNet")
plt.plot(t, lines[4], label="Switching Filter PointNet")

plt.xlabel('Time (s)')
plt.ylabel('Error (m)')
plt.title('')
plt.legend()
plt.grid(True)
plt.savefig("error.png")
plt.show()
