import matplotlib
import matplotlib.pyplot as plt
import numpy as np

data = ""
with open("losslog2", "r") as f:
	data = f.read()

data = data.split("\n")
data = filter(None, data)
data = [float(i) for i in data]

t = np.arange(0, len(data), 1)

fig, ax = plt.subplots()
ax.plot(t, data)

ax.set(xlabel='Frame', ylabel='Loss', title='Online learning loss')
ax.grid()

fig.savefig("test2.png")
plt.show()

