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

# Find the experiment type 
experiment_type = sys.argv[1]

truth = pd.read_csv('experiment-full/true-sin-wave.txt', delimiter=',').to_numpy()
param1 = pd.read_csv('experiment-full/sinwave-5.txt', delimiter=',').to_numpy()[:, 0]
param2 = pd.read_csv('experiment-full/sinwave-6.txt', delimiter=',').to_numpy()[:, 0]
param3 = pd.read_csv('experiment-full/sinwave-7.txt', delimiter=',').to_numpy()[:, 0]
param4 = pd.read_csv('experiment-full/sinwave-8.txt', delimiter=',').to_numpy()[:, 0]
param5 = pd.read_csv('experiment-full/sinwave-100.txt', delimiter=',').to_numpy()[:, 0]

data = np.array([param5, param1, param2, param3, param4])
#parameters = [ 0.001, 0.005, 0.05, 0.08 ]
parameters = [100, 500, 1000, 2000, 3000]
#parameters = [0.01, 0.02, 0.06, 0.1]
color_palette = ['royalblue', 'darkorange', 'crimson', 'limegreen', 'cyan']
xs = np.linspace(0, 900, len(truth))

for i in range(len(data)):
    plt.plot(xs, data[i], label='{} = {}'.format(experiment_type, parameters[i]), alpha=0.7, c=color_palette[i])

plt.plot(xs, truth[:, 0], label="Ground Truth", color='black', linewidth=2.0)

plt.ylabel('X-coordinate')
plt.xlabel('Time [s]')
plt.xlim(left=0)
plt.ylim(bottom=min(data.flatten()))
plt.title('True Movement vs. Predicted Movement Along X-Axis')
plt.legend(loc=2, prop={'size': 12})

# Save the file 
fig.set_size_inches(7.2, 5)
plt.savefig('report-plots/svg/sinwave-{experiment}.svg'.format(experiment=experiment_type), dpi=1000, bbox_inches='tight')
plt.show()
