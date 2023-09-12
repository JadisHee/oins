import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv('./2/record_8.csv').values
# 行驶状态
state = data[:,0]

# 编码器速度
encoder_velocity = data[:,1]

# 惯导速度
imu_velocity = data[:,2]

# slam位移
slam_displacement = data[:,3]

# 惯导位移
imu_displacement = data[:,4]

# 编码器位移
encoder_displacement = data[:,5]

# 融合定位
fusion_displacement = data[:,6]

d = (slam_displacement[1500] - fusion_displacement[1500])/slam_displacement[1500]

print(d)

""" plt.figure(1)
plt.plot(state,label='the movement state')
plt.plot(imu_velocity,label='imu_velocity')
plt.plot(imu_displacement,label='imu_displacement')
plt.legend()
plt.savefig('./2/8/0_Ins.jpg')

plt.figure(2)
plt.plot(state,label='the movement state')
plt.plot(encoder_velocity,label='encoder_velocity')
plt.plot(encoder_displacement,label='encoder_displacement')
# plt.plot(d_encoder_velocity,label='d_encoder_velocity')
plt.legend()
plt.savefig('./2/8/1_encoder.jpg')

plt.figure(3)
plt.plot(slam_displacement,label='slam_displacement')
plt.plot(imu_displacement,label='imu_displacement')
plt.plot(encoder_displacement,label='encoder_displacement')
plt.plot(fusion_displacement,label='fusion_displacement')
plt.legend()
plt.savefig('./2/8/2_fusion.jpg') """
