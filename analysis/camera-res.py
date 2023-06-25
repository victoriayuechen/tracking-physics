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
fig, ax = plt.subplots()

true = pd.read_csv('experiment-full/true-translation.txt', delimiter=',').to_numpy()[:, 0]
res_1 = pd.read_csv('experiment-full/res1.txt', delimiter=',').to_numpy()[:, 0]
res_2 = pd.read_csv('experiment-full/res2.txt', delimiter=',').to_numpy()[:, 0]
res_3 = pd.read_csv('experiment-full/res3.txt', delimiter=',').to_numpy()[:, 0]
res_4 = pd.read_csv('experiment-full/res4.txt', delimiter=',').to_numpy()[:, 0]
res_5 = pd.read_csv('experiment-full/res5.txt', delimiter=',').to_numpy()[:, 0]
true = true[:len(res_1)]

cams = [res_1, res_2, res_3, res_4, res_5]
res_labels = ['30 x 40', '40 x 50', '50 x 60', '60 x 70', '70 x 80']
l2norms = []

for c in cams:
    l2norms.append(np.abs(c - true))


time_1 = pd.read_csv('experiment-full/res1-timing.txt', delimiter=',').to_numpy()[:, 0]
time_2 = pd.read_csv('experiment-full/res2-timing.txt', delimiter=',').to_numpy()[:, 0]
time_3 = pd.read_csv('experiment-full/res3-timing.txt', delimiter=',').to_numpy()[:, 0]
time_4 = pd.read_csv('experiment-full/res4-timing.txt', delimiter=',').to_numpy()[:, 0]
time_5 = pd.read_csv('experiment-full/res5-timing.txt', delimiter=',').to_numpy()[:, 0]
times = [time_1, time_2, time_3, time_4, time_5]
ax2 = ax.twinx()

ax2.set_ylabel('Average Frame Processing Time [ms]')
bplot2 = ax2.boxplot(times, labels=['', '', '', '', ''], patch_artist=True, vert=True, showfliers=False)

bplot1 = ax.boxplot(l2norms, vert=True, labels=res_labels, patch_artist=True, showfliers=False)

for patch in bplot1['boxes']:
    patch.set_alpha(0.6)
    patch.set_facecolor('royalblue')

plt.setp(bplot1['whiskers'], color='blue')
plt.setp(bplot1['caps'], color='blue')

for patch in bplot2['boxes']:
    patch.set_alpha(0.6)
    patch.set_facecolor('lightgreen')

plt.setp(bplot2['whiskers'], color='green')
plt.setp(bplot2['caps'], color='green')



ax.yaxis.grid(True)
ax.set_xlabel('Camera Resolution')
ax.set_ylabel('L2 Norm Error')
ax.set_title('Comparison with Camera Resolution and Accuracy')

ax.yaxis.label.set_color('royalblue')
ax.tick_params(axis='y',colors='blue')

ax2.yaxis.label.set_color('green')
ax2.tick_params(axis='y',colors='green')

fig.savefig('report-plots/svg/res-accuracy.svg')

