fn = "sensor_fusion-9-stdout.log"

data = []
with open(fn, "r") as f:
	data = f.read()
data = data.split("\n")
data = filter(None, data)
#Las/Rad/Deep/KF/SF 1.319630 0.366953 0.366953 -nan 0.366953 1

bufs = [[],[],[],[],[]]
slidingWindowLen = 20
vals = []
for lineidx in range(len(data)):
	line = data[lineidx]

	if "nan" in line:
		print("Warning, NAN")
		continue

	line = line.split(" ")

	for i in range(1,8):
		bufs[i-1].append(line[i])

	if len(bufs[0]) > slidingWindowLen:
		for i in range(0,7):
			del bufs[i][0]

	avgs = []
	for i in range(0, 7):
		total = 0
		n = 0
		for j in bufs[i]:
			total += float(j)
			n += 1
		avg = total/n
		avgs.append(str(avg))

	vals.append(" ".join(avgs))

with open(fn + ".a", "w") as f:
	for i in vals:
		f.write(i + "\n")
