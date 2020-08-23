import numpy as np
import matplotlib.pyplot as plt
data = []

with open("./sensor_fusion-8-stdout.log.filtered", "r") as f:
    data = f.read()

data = data.split("\n")
data = filter(None, data)

maxRange = 2
thresholds = np.arange(0.0, maxRange, 0.01)
precisions = []
for i in range(0, 6):
    precisions.append([])

for threshold in thresholds:
    tp = []
    fp = []

    for i in range(0, 6):
        tp.append(0)
        fp.append(0)

    for line in data:
        if "Las/Rad" not in line:
            continue
        line = line.split(" ")

        for i in range(0, 6):
            val = float(line[i+3])
            if val == 0:
                continue

        #las, svm, deep, kf, sf, kfd, sfd
        laspoint = float(line[2])
        for i in range(0, 6):
            val = float(line[i + 3])
            val = abs(val - laspoint)

            if val < threshold:
                tp[i] += 1
            else:
                fp[i] += 1

    #precision or positive predictive power
    #Precision = True Positives / (True Positives + False Positives)
    for i in range(0, 6):
        val = tp[i] / float((tp[i] + fp[i]))
        #print(val)
        precisions[i].append(val)

plt.figure(dpi=200)
axes = plt.gca()
axes.set_xlim([-0.05, 1.50])
axes.set_ylim([-0.05,1.05])
axes.grid()

plt.suptitle('Precision of the framework over distance', fontsize=10)
plt.xlabel('Distance Threshold [m]', fontsize=10)
plt.ylabel('Precision [-]', fontsize=10)

labels = ["SVM", "PN", "SVM-KF", "SVM-SF", "PN-KF", "PN-SF"]

for i in range(0, 6):
    plt.plot(thresholds, precisions[i], label=labels[i])
plt.legend(loc="lower right")
plt.plot(thresholds, thresholds, "--", label="Reference", color="grey")
plt.show()

