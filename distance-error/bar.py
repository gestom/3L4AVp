import matplotlib.pyplot as plt; plt.rcdefaults()
import numpy as np
import matplotlib.pyplot as plt
import os

try:
    os.mkdir("results-charts")
except:
    print("merging with existing files")

files = []
for i in os.walk("./results"):
    files = i[2]

for fn in files:
    
    if "swp" in fn:
        continue

    datas = []
    with open("./results/" + fn, "r") as f:
        datas = f.read()

    datas = datas.split("\n")
    datas = filter(None, datas)
    
    xs = []
    ys = []

    for line in datas:
        line = line.split(" ")
        xs.append(float(line[0]))
        ys.append(float(line[1]))

    plt.bar(xs, ys, width=0.1, alpha=0.7, edgecolor="b")
    plt.ylabel('Samples')
    plt.grid(color='#95a5a6', linestyle='--', linewidth=2, axis='y', alpha=0.7)
    plt.title(fn)
    plt.xlabel("Distance(m)")

    plt.savefig("results-charts/" + fn + ".png")
    plt.show()
