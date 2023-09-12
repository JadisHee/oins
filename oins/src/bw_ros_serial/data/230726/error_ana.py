import pandas as pd
import matplotlib.pyplot as plt
import csv
import numpy as np
import array


def popMaxMin(arg):
    seq = list(arg)
    seq.pop(seq.index(max(seq)))
    seq.pop(seq.index(min(seq)))
    return seq

data = pd.read_csv('error.csv').values



data_1 = [0 for i in range(8)]

for i in range(0,len(data)):
    data_1[i] = data[i,0]


print(data_1)
print(np.mean(data_1))

plt.figure(1)
plt.plot(data,label='error(m/m)')
plt.legend()
plt.show()
