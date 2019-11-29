import h5py
import os
import shutil
import numpy as np
import random
import math

#please leave rotation at zero for now...todo: normalise coords aroudn zero, then apply the rot, then undo norm
rotationAugmentations = 4
flipX = True
flipY = True

datasets = []
for i in os.walk("datasets"):
	if i[0] == 'datasets':
		print i[1]
		datasets = i[1]
		break

datasetname = raw_input("Enter dataset name: ")
if datasetname not in datasets:
		print "Dataset not found"
		sys.exit()
datasetfn = os.path.join("datasets", datasetname, "velocity")
datasetfna = os.path.join("datasets", datasetname, "vel-rot-" + str(rotationAugmentations) + "-flipx-" + str(flipX) + "-flipy-" + str(flipY))

if os.path.isdir(datasetfna):
	shutil.rmtree(datasetfna)
shutil.copytree(datasetfn, datasetfna)

#finally, augment the actual data
data = h5py.File(os.path.join(datasetfn, "data.h5"), "r")
datal = data["label"]
datad = data["data"]

datasetLength = datal.shape[0]
newDatasetLength = datasetLength * (rotationAugmentations + 1)
if flipX:
	newDatasetLength *= 2
if flipY:
	newDatasetLength *= 2

newl = np.zeros((newDatasetLength, datal.shape[1]), datal.dtype)
newd = np.zeros((newDatasetLength, datad.shape[1], datad.shape[2]), datad.dtype)

for frameIndex in xrange(0, len(datad[0])):

	#copy labels
	for augmentationIndex in xrange(0, rotationAugmentations + 1):
		
		horizontalFlipRange = 1
		if flipX:
			horizontalFlipRange *= 2
		if flipY:
			horizontalFlipRange *= 2

		for horizontalIndex in xrange(0, horizontalFlipRange):
			for pointIndex in xrange(0, len(datal[frameIndex])):

				topLevelIndex = (frameIndex * (rotationAugmentations + 1) * horizontalFlipRange)
				topLevelIndex += (augmentationIndex * horizontalFlipRange)
				topLevelIndex += horizontalIndex

				newl[topLevelIndex][pointIndex] = datal[frameIndex][pointIndex]

	#copy and random rotate points
	for augmentationIndex in xrange(0, rotationAugmentations + 1):

		rotation = random.uniform(0, 2 * math.pi)
		for pointIndex in xrange(0, len(datad[frameIndex])):

			insertionPoint = 0
			
			originalPoint = datad[frameIndex][pointIndex]

			x = (math.cos(rotation) * originalPoint[0]) - (math.sin(rotation) * originalPoint[1])
			y = (math.cos(rotation) * originalPoint[1]) + (math.sin(rotation) * originalPoint[0])
			z = originalPoint[2]
			r = originalPoint[3]
			g = originalPoint[4]
			b = originalPoint[5]
			xn = (math.cos(rotation) * originalPoint[6]) - (math.sin(rotation) * originalPoint[7])
			yn = (math.cos(rotation) * originalPoint[7]) + (math.sin(rotation) * originalPoint[6])
			zn = originalPoint[2]

			topLevelIndex = (frameIndex * (rotationAugmentations + 1) * horizontalFlipRange)
			topLevelIndex += (augmentationIndex * horizontalFlipRange)
			topLevelIndex += insertionPoint
			insertionPoint += 1

			tmp = [x, y, z, r, g, b, xn, yn, zn]
			for i in xrange(0, 9):
				newd[topLevelIndex][pointIndex][i] = tmp[i]

			if flipX:
				
				originalPoint = datad[frameIndex][pointIndex]

				x = -((math.cos(rotation) * originalPoint[0]) - (math.sin(rotation) * originalPoint[1]))
				y = (math.cos(rotation) * originalPoint[1]) + (math.sin(rotation) * originalPoint[0])
				z = originalPoint[2]
				r = originalPoint[3]
				g = originalPoint[4]
				b = originalPoint[5]
				xn = 1 - ((math.cos(rotation) * originalPoint[6]) - (math.sin(rotation) * originalPoint[7]))
				yn = (math.cos(rotation) * originalPoint[7]) + (math.sin(rotation) * originalPoint[6])
				zn = originalPoint[2]

				topLevelIndex = (frameIndex * (rotationAugmentations + 1) * horizontalFlipRange)
				topLevelIndex += (augmentationIndex * horizontalFlipRange)
				topLevelIndex += insertionPoint
				insertionPoint += 1

				tmp = [x, y, z, r, g, b, xn, yn, zn]
				for i in xrange(0, 9):
					newd[topLevelIndex][pointIndex][i] = tmp[i]

			if flipY:
				
				originalPoint = datad[frameIndex][pointIndex]

				x = (math.cos(rotation) * originalPoint[0]) - (math.sin(rotation) * originalPoint[1])
				y = -((math.cos(rotation) * originalPoint[1]) + (math.sin(rotation) * originalPoint[0]))
				z = originalPoint[2]
				r = originalPoint[3]
				g = originalPoint[4]
				b = originalPoint[5]
				xn = (math.cos(rotation) * originalPoint[6]) - (math.sin(rotation) * originalPoint[7])
				yn = 1 - ((math.cos(rotation) * originalPoint[7]) + (math.sin(rotation) * originalPoint[6]))
				zn = originalPoint[2]

				topLevelIndex = (frameIndex * (rotationAugmentations + 1) * horizontalFlipRange)
				topLevelIndex += (augmentationIndex * horizontalFlipRange)
				topLevelIndex += insertionPoint
				insertionPoint += 1

				tmp = [x, y, z, r, g, b, xn, yn, zn]
				for i in xrange(0, 9):
					newd[topLevelIndex][pointIndex][i] = tmp[i]

			if flipX and flipY:
				
				originalPoint = datad[frameIndex][pointIndex]

				y = (math.cos(rotation) * originalPoint[0]) - (math.sin(rotation) * originalPoint[1])
				x = (math.cos(rotation) * originalPoint[1]) + (math.sin(rotation) * originalPoint[0])
				z = originalPoint[2]
				r = originalPoint[3]
				g = originalPoint[4]
				b = originalPoint[5]
				yn = (math.cos(rotation) * originalPoint[6]) - (math.sin(rotation) * originalPoint[7])
				xn = (math.cos(rotation) * originalPoint[7]) + (math.sin(rotation) * originalPoint[6])
				zn = originalPoint[2]

				topLevelIndex = (frameIndex * (rotationAugmentations + 1) * horizontalFlipRange)
				topLevelIndex += (augmentationIndex * horizontalFlipRange)
				topLevelIndex += insertionPoint
				insertionPoint += 1

				tmp = [x, y, z, r, g, b, xn, yn, zn]
				for i in xrange(0, 9):
					newd[topLevelIndex][pointIndex][i] = tmp[i]

data.close()

#shuffle new arrays
rng_state = np.random.get_state()
np.random.shuffle(newl)
np.random.set_state(rng_state)
np.random.shuffle(newd)

#write dataset
f = h5py.File(os.path.join(datasetfna, "data.h5"), 'w')
label = f.create_dataset("label", data=newl)
data = f.create_dataset("data", data=newd)
f.close()


datasetfn = os.path.join("datasets", datasetname, "no-velocity")
datasetfna = os.path.join("datasets", datasetname, "novel-rot-" + str(rotationAugmentations) + "-flipx-" + str(flipX) + "-flipy-" + str(flipY))

if os.path.isdir(datasetfna):
	shutil.rmtree(datasetfna)
shutil.copytree(datasetfn, datasetfna)

#finally, augment the actual data
data = h5py.File(os.path.join(datasetfn, "data.h5"), "r")
datal = data["label"]
datad = data["data"]

datasetLength = datal.shape[0]
newDatasetLength = datasetLength * (rotationAugmentations + 1)
if flipX:
	newDatasetLength *= 2
if flipY:
	newDatasetLength *= 2

newl = np.zeros((newDatasetLength, datal.shape[1]), datal.dtype)
newd = np.zeros((newDatasetLength, datad.shape[1], datad.shape[2]), datad.dtype)

for frameIndex in xrange(0, len(datad[0])):

	#copy labels
	for augmentationIndex in xrange(0, rotationAugmentations + 1):
		
		horizontalFlipRange = 1
		if flipX:
			horizontalFlipRange *= 2
		if flipY:
			horizontalFlipRange *= 2

		for horizontalIndex in xrange(0, horizontalFlipRange):
			for pointIndex in xrange(0, len(datal[frameIndex])):

				topLevelIndex = (frameIndex * (rotationAugmentations + 1) * horizontalFlipRange)
				topLevelIndex += (augmentationIndex * horizontalFlipRange)
				topLevelIndex += horizontalIndex

				newl[topLevelIndex][pointIndex] = datal[frameIndex][pointIndex]

	#copy and random rotate points
	for augmentationIndex in xrange(0, rotationAugmentations + 1):

		rotation = random.uniform(0, 2 * math.pi)
		for pointIndex in xrange(0, len(datad[frameIndex])):

			insertionPoint = 0
			
			originalPoint = datad[frameIndex][pointIndex]

			x = (math.cos(rotation) * originalPoint[0]) - (math.sin(rotation) * originalPoint[1])
			y = (math.cos(rotation) * originalPoint[1]) + (math.sin(rotation) * originalPoint[0])
			z = originalPoint[2]
			r = originalPoint[3]
			g = originalPoint[4]
			b = originalPoint[5]
			xn = (math.cos(rotation) * originalPoint[6]) - (math.sin(rotation) * originalPoint[7])
			yn = (math.cos(rotation) * originalPoint[7]) + (math.sin(rotation) * originalPoint[6])
			zn = originalPoint[2]

			topLevelIndex = (frameIndex * (rotationAugmentations + 1) * horizontalFlipRange)
			topLevelIndex += (augmentationIndex * horizontalFlipRange)
			topLevelIndex += insertionPoint
			insertionPoint += 1

			tmp = [x, y, z, r, g, b, xn, yn, zn]
			for i in xrange(0, 9):
				newd[topLevelIndex][pointIndex][i] = tmp[i]

			if flipX:
				
				originalPoint = datad[frameIndex][pointIndex]

				x = -((math.cos(rotation) * originalPoint[0]) - (math.sin(rotation) * originalPoint[1]))
				y = (math.cos(rotation) * originalPoint[1]) + (math.sin(rotation) * originalPoint[0])
				z = originalPoint[2]
				r = originalPoint[3]
				g = originalPoint[4]
				b = originalPoint[5]
				xn = 1 - ((math.cos(rotation) * originalPoint[6]) - (math.sin(rotation) * originalPoint[7]))
				yn = (math.cos(rotation) * originalPoint[7]) + (math.sin(rotation) * originalPoint[6])
				zn = originalPoint[2]

				topLevelIndex = (frameIndex * (rotationAugmentations + 1) * horizontalFlipRange)
				topLevelIndex += (augmentationIndex * horizontalFlipRange)
				topLevelIndex += insertionPoint
				insertionPoint += 1

				tmp = [x, y, z, r, g, b, xn, yn, zn]
				for i in xrange(0, 9):
					newd[topLevelIndex][pointIndex][i] = tmp[i]

			if flipY:
				
				originalPoint = datad[frameIndex][pointIndex]

				x = (math.cos(rotation) * originalPoint[0]) - (math.sin(rotation) * originalPoint[1])
				y = -((math.cos(rotation) * originalPoint[1]) + (math.sin(rotation) * originalPoint[0]))
				z = originalPoint[2]
				r = originalPoint[3]
				g = originalPoint[4]
				b = originalPoint[5]
				xn = (math.cos(rotation) * originalPoint[6]) - (math.sin(rotation) * originalPoint[7])
				yn = 1 - ((math.cos(rotation) * originalPoint[7]) + (math.sin(rotation) * originalPoint[6]))
				zn = originalPoint[2]

				topLevelIndex = (frameIndex * (rotationAugmentations + 1) * horizontalFlipRange)
				topLevelIndex += (augmentationIndex * horizontalFlipRange)
				topLevelIndex += insertionPoint
				insertionPoint += 1

				tmp = [x, y, z, r, g, b, xn, yn, zn]
				for i in xrange(0, 9):
					newd[topLevelIndex][pointIndex][i] = tmp[i]

			if flipX and flipY:
				
				originalPoint = datad[frameIndex][pointIndex]

				y = (math.cos(rotation) * originalPoint[0]) - (math.sin(rotation) * originalPoint[1])
				x = (math.cos(rotation) * originalPoint[1]) + (math.sin(rotation) * originalPoint[0])
				z = originalPoint[2]
				r = originalPoint[3]
				g = originalPoint[4]
				b = originalPoint[5]
				yn = (math.cos(rotation) * originalPoint[6]) - (math.sin(rotation) * originalPoint[7])
				xn = (math.cos(rotation) * originalPoint[7]) + (math.sin(rotation) * originalPoint[6])
				zn = originalPoint[2]

				topLevelIndex = (frameIndex * (rotationAugmentations + 1) * horizontalFlipRange)
				topLevelIndex += (augmentationIndex * horizontalFlipRange)
				topLevelIndex += insertionPoint
				insertionPoint += 1

				tmp = [x, y, z, r, g, b, xn, yn, zn]
				for i in xrange(0, 9):
					newd[topLevelIndex][pointIndex][i] = tmp[i]

data.close()

#shuffle new arrays
rng_state = np.random.get_state()
np.random.shuffle(newl)
np.random.set_state(rng_state)
np.random.shuffle(newd)

#write dataset
f = h5py.File(os.path.join(datasetfna, "data.h5"), 'w')
label = f.create_dataset("label", data=newl)
data = f.create_dataset("data", data=newd)
f.close()
