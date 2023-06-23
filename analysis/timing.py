import sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Font size and .tex background
params = {'text.usetex' : True,
          'font.size' : 15,
          'font.family' : 'lmodern'
          }
plt.rcParams.update(params) 
fig = plt.figure()

translation = []
acceleration = []
sinwave = []

param = [300, 800, 1000, 1500, 2000, 3000]
for i in range(1, 7):
    translation.append(pd.read_csv('experiment-full/timing-{num}.txt'.format(num=i), delimiter=',').to_numpy())
    acceleration.append(pd.read_csv('experiment-full/timing-acc-{num}.txt'.format(num=i), delimiter=',').to_numpy())
    sinwave.append(pd.read_csv('experiment-full/timing-sw-{num}.txt'.format(num=i), delimiter=',').to_numpy())

ys_translation = []
ys_acceleration = []
ys_sinwave = []

for i in range(len(translation)):
    ys_translation.append(np.mean(translation[i]))
    ys_acceleration.append(np.mean(acceleration[i]))
    ys_sinwave.append(np.mean(sinwave[i]))

plt.plot(param, ys_acceleration, marker='o', c='darkorange', label='Acceleration')
plt.plot(param, ys_translation, marker='o', c='royalblue', label='Translation')
plt.plot(param, ys_sinwave, marker='o', c='crimson', label='Sin Wave')

plt.ylabel('Average Frame Processing Time [ms]')
plt.xlabel('Number of Particles')
plt.xlim(left=0)
plt.ylim(bottom=0)
plt.legend(loc=2, prop={'size': 12})
plt.title('Particle Count and Average Processing Time')

# Save the file 
fig.set_size_inches(7.2, 5)
plt.savefig('report-plots/pNo-Time.svg', dpi=1000, bbox_inches='tight')
plt.show()

# # Frame rate of pcl vs. unity

# pcl_time1 = pd.read_csv('experiment-full/timing-1.txt', delimiter=',').to_numpy() # this one
# pcl_time2 = pd.read_csv('experiment-full/timing-100.txt', delimiter=',').to_numpy() # this one
# unity_time = (1 / 30) * np.ones_like(pcl_time1)

# ys_pcl1 = np.cumsum((1 / 1000) * pcl_time1)
# ys_pcl2 = np.cumsum((1 / 1000) * pcl_time2)
# ys_unity = np.cumsum(unity_time)
# ys_pcl1[0] = 0
# ys_pcl2[0] = 0
# ys_unity[0] = 0
# xs = range(0, 899)

# plt.plot(ys_pcl1, xs, label='Particle Count = 300', c='royalblue', linewidth=2.5)
# plt.plot(ys_pcl2, xs, label='Particle Count = 100', c='darkorange', linewidth=2.5)
# plt.plot(ys_unity, xs, label='Depth Camera Frame Rate', c='black', linewidth=2.5)

# plt.ylabel('Number of Frames')
# plt.xlabel('Time [s]')
# plt.xlim(left=0)
# plt.ylim(bottom=0)
# plt.title('Processing Rate of Tracking vs. Frame Rate Depth Camera')
# plt.legend(loc=2, prop={'size': 12})
# fig.set_size_inches(7.2, 5)
# plt.savefig('report-plots/svg/fps-compare.svg', dpi=1000, bbox_inches='tight')
# plt.show()

