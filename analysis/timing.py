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

# tracking_time_1 = pd.read_csv('experiment-full/timing-1.txt', delimiter=',').to_numpy() 
# tracking_time_2 = pd.read_csv('experiment-full/timing-2.txt', delimiter=',').to_numpy() 
# tracking_time_3 = pd.read_csv('experiment-full/timing-3.txt', delimiter=',').to_numpy() 
# tracking_time_4 = pd.read_csv('experiment-full/timing-4.txt', delimiter=',').to_numpy() 
# tracking_time_5 = pd.read_csv('experiment-full/timing-5.txt', delimiter=',').to_numpy() # this one
# tracking_time_6 = pd.read_csv('experiment-full/timing-6.txt', delimiter=',').to_numpy() 
# param = [300, 800, 1000, 1500, 2000, 3000]

# times = [tracking_time_1, tracking_time_2, tracking_time_3, tracking_time_4, tracking_time_5, tracking_time_6]
# ys = []
# for i in range(len(times)):
#     ys.append(np.sum(times[i]))

# plt.plot(param, ys, marker='o', c='royalblue')

# plt.ylabel('Time [ms]')
# plt.xlabel('Number of Particles')
# plt.xlim(left=0)
# plt.ylim(bottom=0)
# plt.title('Particle Count and Computational Efficiency')
# plt.legend(loc=2, prop={'size': 12})

# # Save the file 
# fig.set_size_inches(7.2, 5)
# plt.savefig('report-plots/svg/pNo-Time.svg', dpi=1000, bbox_inches='tight')
# plt.show()

# # Frame rate of pcl vs. unity

pcl_time1 = pd.read_csv('experiment-full/timing-1.txt', delimiter=',').to_numpy() # this one
pcl_time2 = pd.read_csv('experiment-full/timing-100.txt', delimiter=',').to_numpy() # this one
unity_time = (1 / 30) * np.ones_like(pcl_time1)

ys_pcl1 = np.cumsum((1 / 1000) * pcl_time1)
ys_pcl2 = np.cumsum((1 / 1000) * pcl_time2)
ys_unity = np.cumsum(unity_time)
ys_pcl1[0] = 0
ys_pcl2[0] = 0
ys_unity[0] = 0
xs = range(0, 899)

plt.plot(xs, ys_pcl1, label='PCL tracking time, N = 300', c='royalblue', linewidth=2.5)
plt.plot(xs, ys_pcl2, label='PCL tracking time, N = 100', c='darkorange', linewidth=2.5)
plt.plot(xs, ys_unity, label='Unity Depth Camera FPS', c='crimson', linewidth=2.5)

plt.ylabel('Time [s]')
plt.xlabel('Number of Frames')
plt.xlim(left=0)
plt.ylim(bottom=0)
plt.title('Processing Rate of Tracking vs. Depth Camera')
plt.legend(loc=2, prop={'size': 12})
fig.set_size_inches(7.2, 5)
plt.savefig('report-plots/svg/fps-compare.svg')
plt.show()

