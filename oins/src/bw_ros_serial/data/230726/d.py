import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import math

data = pd.read_csv('d.csv').values

dt = 0.016

slam_displacement = data[:,0]
print(slam_displacement)
encoder_displacement = data[:,1]
times = data[:,2]
print(times)
times = times * dt
d_displacement = slam_displacement - encoder_displacement
print(d_displacement)
delta_velocity = np.zeros_like(d_displacement)
for i in range(0,len(d_displacement)):

    delta_velocity[i] = d_displacement[i] / times[i]

print(delta_velocity)

mean = sum(delta_velocity)/len(delta_velocity)
print(mean)
'''
3.13987,2.96331,0.0
3.24112,3.05694,0.0
3.18091,2.98671,0.0
3.19190,3.00271,0.0
3.29187,3.09466,0.0
3.36598,3.15385,0.0
3.35036,3.15187,0.0
'''