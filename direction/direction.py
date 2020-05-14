import math
import numpy as np
import matplotlib.pyplot as plt

fn = "sensor_fusion-9-stdout.log"

data = []
with open(fn, "r") as f:
    data = f.read()
data = data.split("\n")
data = filter(None, data)

pi = 3.14159265358979
rad = 360/(2*pi)

bearings = []

with open("bearing.txt", "w") as f:
    for line in data:
        line = line.split(" ")
        try:
            x1 = float(line[-4])
            y1 = float(line[-3])
            x2 = float(line[-2])
            y2 = float(line[-1])
            if(x1 == x2 or y1 == y2):
                print("add same axes handling")
            bearing = math.atan2(x2 - x1, y1 - y2)
            bearing = -(bearing - pi) * rad

            f.write(str(bearing) + "\n")
            bearings.append(float(bearing))

        except:
            print("error on line of file, ignoring:")
            print(line)

print("bearings written to bearings.txt")

bins = list(range(0, 360, 1))
bins.append(360)
plt.hist(bearings, bins=bins)
plt.show()
