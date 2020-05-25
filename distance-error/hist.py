import numpy as np

fn = "./sensor_fusion-9-stdout.filtered.log"

binSize = 0.1
start = 0
finish = 2
skipFirstLines = 2500

data = []
with open(fn, "r") as f:
    data = f.read()

data = data.split("\n")
data = list(filter(None, data))

hists = []

for i in range(1, 8):
    blank = list(np.arange(start, finish+binSize, binSize))
    d = {}
    for j in blank:
        d[j] = 0
    hists.append(d)

lineCtr = 0
for line in data:
    lineCtr += 1
    if lineCtr < skipFirstLines:
        continue

    line = line.split(" ")
    for token in range(1, 8):
        bin = float(line[token]) - (float(line[token]) % binSize)
        if bin > finish:
            bin = finish
        hists[token-1][bin] += 1

import os
try:
    os.mkdir("results")
except:
    pass

labels = data[-1].split("/")[0:7]

print(labels)

for i in range(0, len(hists)):
    print(labels[i], hists[i])

    with open("results/" + labels[i], "w") as f:

        srted = {k: v for k, v in sorted(hists[i].items(), key=lambda item: item[0])}
        
        for key, value in srted.items():
            f.write(str(key) + " " + str(value) + "\n")

