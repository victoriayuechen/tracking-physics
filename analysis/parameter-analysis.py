import sys
import pandas as pd
import matplotlib
matplotlib.use('svg')
import matplotlib.pyplot as plt
plt.rcParams.update({
    'font.size':11, 
    "font.family": "serif",
    # Use LaTeX default serif font.
    "font.serif": [],
    # Use specific cursive fonts.
    "font.cursive": ["Comic Neue", "Comic Sans MS"],
})

from mpltools import style
from mpltools import layout

import numpy as np

# Find the experiment type 
experiment_type = sys.argv[1]

truth = pd.read_csv('experiment-full/true-sin-wave.txt', delimiter=',').to_numpy()
param1 = pd.read_csv('experiment-full/sinwave-9.txt', delimiter=',').to_numpy()[:, 0]
param2 = pd.read_csv('experiment-full/sinwave-10.txt', delimiter=',').to_numpy()[:, 0]
param3 = pd.read_csv('experiment-full/sinwave-11.txt', delimiter=',').to_numpy()[:, 0]
param4 = pd.read_csv('experiment-full/sinwave-12.txt', delimiter=',').to_numpy()[:, 0]

data = np.array([param1, param2, param3, param4])
#parameters = [ 0.001, 0.005, 0.05, 0.08 ]
#parameters = [500, 1000, 2000, 3000 ]
parameters = [0.01, 0.02, 0.06, 0.1]
color_palette = ['royalblue', 'darkorange', 'crimson', 'limegreen']
xs = np.linspace(0, 9, len(truth))

for i in range(len(data)):
    plt.plot(xs, data[i], label='{} = {}'.format(experiment_type, parameters[i]), alpha=0.7, c=color_palette[i])

plt.plot(xs, truth[:, 0], label="Ground Truth", color='black', linewidth=2.0)

plt.ylabel('X-coordinate')
plt.xlabel('Time [s]')
plt.xlim(left=0)
plt.ylim(bottom=min(data.flatten()))
plt.title('True Movement vs. Actual Movement Along X-Axis')
plt.legend()
# plt.savefig('report-plots/acceleration-{experiment}.pgf'.format(experiment=experiment_type), format='pgf')
# Set size of figure, to make font larger.
plt.savefig('report-plots/svg/sinwave-{experiment}.svg'.format(experiment=experiment_type))
plt.show()
