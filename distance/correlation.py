from numpy import mean
from numpy import std
from numpy.random import randn
from numpy.random import seed
from numpy import cov
from matplotlib import pyplot
from scipy.stats import pearsonr
from scipy.stats import spearmanr

seed(69)

data = []
with open("./sensor_fusion-8-stdout.log", "r") as f:
    data = f.read()

data = filter(None, data.split("\n"))

distances = []
errors = []

for line in data:
    line = line.split(" ")

    distance = (float(line[-5])*float(line[-5])+float(line[-4])*float(line[-4]))**0.5
    distances.append(distance)

    SVMerror = float(line[3])
    Deeperror = float(line[4])

    errors.append(SVMerror)

data1 = distances
data2 = errors

print('data1: mean=%.3f stdv=%.3f' % (mean(data1), std(data1)))
print('data2: mean=%.3f stdv=%.3f' % (mean(data2), std(data2)))

covariance = cov(data1, data2)
print(covariance)

corr, _ = pearsonr(data1, data2)
print('Pearsons correlation: %.3f' % corr)

corr, _ = spearmanr(data1, data2)
print('Spearmans correlation: %.3f' % corr)

