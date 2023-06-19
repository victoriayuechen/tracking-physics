import sys
import pandas as pd
import matplotlib
matplotlib.use('pgf')
import matplotlib.pyplot as plt
plt.rcParams.update({
    "font.family": "serif",
    # Use LaTeX default serif font.
    "font.serif": [],
    # Use specific cursive fonts.
    "font.cursive": ["Comic Neue", "Comic Sans MS"],
})

import numpy as np

from mpltools import style
from mpltools import layout

tracking_time_1 = pd.read_csv('experiment-full/timing-1.txt', delimiter=',').to_numpy() 
tracking_time_2 = pd.read_csv('experiment-full/timing-2.txt', delimiter=',').to_numpy() 
tracking_time_3 = pd.read_csv('experiment-full/timing-3.txt', delimiter=',').to_numpy() 
tracking_time_4 = pd.read_csv('experiment-full/timing-4.txt', delimiter=',').to_numpy() 
tracking_time_5 = pd.read_csv('experiment-full/timing-5.txt', delimiter=',').to_numpy() # this one
tracking_time_6 = pd.read_csv('experiment-full/timing-6.txt', delimiter=',').to_numpy() 
param = [300, 800, 1000, 1500, 2000, 3000]

times = [tracking_time_1, tracking_time_2, tracking_time_3, tracking_time_4, tracking_time_5, tracking_time_6]
ys = []
for i in range(len(times)):
    ys.append(np.sum(times[i]))

plt.plot(param, ys, marker='o', c='royalblue')

plt.ylabel('Time [ms]')
plt.xlabel('Number of Particles')
plt.xlim(left=0)
plt.ylim(bottom=0)
plt.title('Particle Count and Computational Efficiency')
plt.savefig('report-plots/pNo-Time.pgf', format='pgf')
plt.savefig('report-plots/svg/pNo-Time.svg')
plt.show()

# # Frame rate of pcl vs. unity

# pcl_time2000 = pd.read_csv('experiment-full/timing-5.txt', delimiter=',').to_numpy() # this one
# pcl_time300 = pd.read_csv('experiment-full/timing-1.txt', delimiter=',').to_numpy() # this one
# unity_time = (1 / 30) * np.ones_like(pcl_time2000)

# ys_pcl2000 = np.cumsum((1 / 1000) * pcl_time2000)
# ys_pcl300 = np.cumsum((1 / 1000) * pcl_time300)
# ys_unity = np.cumsum(unity_time)
# ys_pcl2000[0] = 0
# ys_pcl300[0] = 0
# ys_unity[0] = 0
# xs = range(0, 899)

# plt.plot(xs, ys_pcl2000, label='PCL tracking time, N = 2000', c='royalblue', linewidth=2.5)
# plt.plot(xs, ys_pcl300, label='PCL tracking time, N = 300', c='darkorange', linewidth=2.5)
# plt.plot(xs, ys_unity, label='Unity Depth Camera FPS', c='crimson', linewidth=2.5)

# plt.ylabel('Time [s]')
# plt.xlabel('Number of Frames')
# plt.xlim(left=0)
# plt.ylim(bottom=0)
# plt.title('Processing Rate of Tracking vs. Depth Camera')
# plt.legend()
# plt.savefig('report-plots/fps-compare.pgf', format='pgf')
# plt.savefig('report-plots/svg/fps-compare.svg')
# plt.show()

