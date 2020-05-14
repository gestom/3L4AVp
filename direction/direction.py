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
mags = []

with open("bearing.txt", "w") as f:
    f.write("bearing magnitude")
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

            mag = (x2-x1)*(x2-x1)*(y2-y1)*(y2-y1)
            mag = mag**0.5
            mags.append(float(mag))

            f.write(str(bearing) + " " + str(mag) + "\n")
            bearings.append(float(bearing))

        except:
            print("error on line of file, ignoring:")
            print(line)

print("bearings written to bearings.txt")


#without taking into account magnitude:
#bins = list(range(0, 360, 1))
#bins.append(360)
#plt.hist(bearings, bins=bins)
#plt.show()

#with taking into account:
binSize = 4
binBoundaries = list(range(0, 360, binSize))
binEnds = list(range(binSize, 360 + binSize, binSize))
bins = []
for i in binBoundaries:
    bins.append(0)

for bearing in bearings:
    for j in range(0, len(binBoundaries)):
        if bearing > binBoundaries[j] and bearing < binEnds[j]:
            bins[j] += mag

binMids = []
current = binSize / 2
for i in range(0, len(binBoundaries)):
    binMids.append(current)
    current += binSize

plt.scatter(binMids, bins)
plt.show()
