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
plt.style.use('seaborn-v0_8-whitegrid')

tracking_time_1 = pd.read_csv('experiment-full/timing-1.txt', delimiter=',').to_numpy() 
tracking_time_2 = pd.read_csv('experiment-full/timing-2.txt', delimiter=',').to_numpy() 
tracking_time_3 = pd.read_csv('experiment-full/timing-3.txt', delimiter=',').to_numpy() 
tracking_time_4 = pd.read_csv('experiment-full/timing-4.txt', delimiter=',').to_numpy() 
tracking_time_5 = pd.read_csv('experiment-full/timing-5.txt', delimiter=',').to_numpy() 
tracking_time_6 = pd.read_csv('experiment-full/timing-6.txt', delimiter=',').to_numpy() 
param = [300, 800, 1000, 1500, 2000, 3000]

times = [tracking_time_1, tracking_time_2, tracking_time_3, tracking_time_4, tracking_time_5, tracking_time_6]
ys = []
for i in range(len(times)):
    ys.append(np.sum(times[i]))

plt.plot(param, ys, marker='o')

plt.ylabel('Time [s]')
plt.xlabel('Number of Particles')
plt.xlim(left=0)
plt.ylim(bottom=0)
plt.title('Particle Count and Computational Efficiency')
plt.legend()
plt.savefig('report-plots/pNo-Time.pgf', format='pgf')
plt.savefig('report-plots/svg/pNo-Time.svg')
plt.show()