import random

messageDB = range(0, 50) 
#1/10 2/10 3/10 4/10
#1/10 3/10 6/10 10/10
batchSize = 1

def probMax(x):
		if x == 1:
				return 1
		else:
				return x + probMax(x - 1)

def getBatches():

	batch = []
	labels = []

	# print(range(batchSize))
	for _ in range(batchSize):

			idx = random.randint(0, probMax(len(messageDB)) - 1)
			# print("idx" + str(idx))
			numerator = 0

			for i in range(len(messageDB)):
				numerator += i + 1
				# print(idx, i+1, numerator)
				if idx <= numerator:
						# print("appending")
						batch.append(messageDB[i])
						labels.append(messageDB[i])
						break

	return (batch, labels)

sum=0
total = 0
for i in range(0, 15):

	val = getBatches()[0]
	total += int(val[0])  

	print(val)

print(val[0]/float(total))