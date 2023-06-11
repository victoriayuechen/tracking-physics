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

# Find the experiment type 
experiment_type = sys.argv[1]

truth = pd.read_csv('experiment-full/true-zigzag.txt', delimiter=',').to_numpy()
param1 = pd.read_csv('experiment-full/zigzag-5.txt', delimiter=',').to_numpy()[:, 0]
param2 = pd.read_csv('experiment-full/zigzag-6.txt', delimiter=',').to_numpy()[:, 0]
param3 = pd.read_csv('experiment-full/zigzag-7.txt', delimiter=',').to_numpy()[:, 0]
param4 = pd.read_csv('experiment-full/zigzag-8.txt', delimiter=',').to_numpy()[:, 0]

data = np.array([param1, param2, param3, param4])
#parameters = [ 0.001, 0.005, 0.05, 0.08 ]
parameters = [500, 1000, 2000, 3000 ]
#parameters = [0.01, 0.02, 0.06, 0.1]
color_palette = ['royalblue', 'darkorange', 'crimson', 'limegreen']
xs = np.linspace(0, 9, len(truth))

plt.plot(xs, truth[:, 0], label="Ground Truth", color='black', linewidth=2.0)
for i in range(len(data)):
    plt.plot(xs, data[i], label='{} = {}'.format(experiment_type, parameters[i]), linestyle='--', alpha=0.7, color=color_palette[i])

plt.ylabel('X-coordinate')
plt.xlabel('Time [s]')
plt.xlim(left=0)
plt.ylim(bottom=min(data.flatten()))
plt.title('True Movement vs. Actual Movement Along X-Axis')
plt.legend()
plt.savefig('report-plots/zigzag-{experiment}.pgf'.format(experiment=experiment_type), format='pgf')
plt.savefig('report-plots/svg/zigzag-{experiment}.svg'.format(experiment=experiment_type))
plt.show()
