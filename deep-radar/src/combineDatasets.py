import h5py
import numpy as np
import os
import sys

datasets = []
for i in os.walk("datasets"):
	if i[0] == 'datasets':
		print i[1]
		datasets = i[1]
		break

datasetAName = raw_input("Select first dataset name: ")
if datasetAName not in datasets:
		print "Dataset not found"
		sys.exit()
datasetAFN = os.path.join("datasets", datasetAName)

datasetBName = raw_input("Select second dataset name: ")
if datasetBName not in datasets:
		print "Dataset not found"
		sys.exit()
datasetBFN = os.path.join("datasets", datasetBName)

subsets = []
for i in os.walk(datasetAFN):
	if i[0] == datasetAFN:
		print i[1]
		subsets = i[1]
subsetName = raw_input("Select dataset subset: ")
if subsetName not in subsets:
	print("Subset not found")
	sys.exit()
subsetsB = []
for i in os.walk(datasetBFN):
	if i[0] == datasetBFN:
		subsetsB = i[1]
if subsetName not in subsetsB:
	print("Subset not found in both datasets")
	sys.exit()

datasetAFN = os.path.join(datasetAFN, subsetName)
datasetBFN = os.path.join(datasetBFN, subsetName)
datasetCFN = os.path.join("datasets", "combined-" + datasetAName + "-" + datasetBName, subsetName)
os.makedirs(datasetCFN)

# #arrange class names
# with open(os.path.join(datasetAFN, "")

a = h5py.File(os.path.join(datasetAFN, "data.h5"), "r")
b = h5py.File(os.path.join(datasetBFN, "data.h5"), "r")
c = os.path.join(datasetCFN, "data.h5")

ad = a.get("data").value
al = a.get("label").value

bd = b.get("data").value
bl = b.get("label").value

for i in xrange(0, len(bl)):
	for j in xrange(0, len(bl[i])):
		if bl[i][j] == 1:
			bl[i][j] = 2

if al.shape[1] != bl.shape[1]:
	print("Warning: Truncating tensor shape to match")

tensorShape = min(al.shape[1], bl.shape[1])

ad = ad[:,:tensorShape,:]
al = al[:,:tensorShape]
bd = bd[:,:tensorShape,:]
bl = bl[:,:tensorShape]

cd = np.concatenate((ad, bd))
cl = np.concatenate((al, bl))

rng_state = np.random.get_state()
np.random.shuffle(cd)
np.random.set_state(rng_state)
np.random.shuffle(cl)

f = h5py.File(c, 'w')
label = f.create_dataset("label", data=cl)
data = f.create_dataset("data", data=cd)
f.close()

print("Done")
