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
regular = pd.read_csv('experiment-full/translation-222.txt', delimiter=',').to_numpy()[:, 0]
kld = pd.read_csv('experiment-full/translation-2.txt', delimiter=',').to_numpy()[:, 0]
true = pd.read_csv('experiment-full/true-translation.txt', delimiter=',').to_numpy()[:, 0]

# Shorten the axis
kld = kld[:len(regular)]
true = true[:len(regular)]
xs = np.linspace(0, 900, len(kld))

plt.plot(xs, regular, label='Regular Particle Filter', c='royalblue')
plt.plot(xs, kld, label='KLD Adaptive Filter', c='darkorange')
plt.plot(xs, true, label='Ground Truth', c='black', linewidth=2.5)

plt.xlim(left=0)
plt.ylabel('X-position')
plt.xlabel('Time [s]')
plt.title('Comparison of Particle Filters')
# Set size of legend
plt.legend(loc=2, prop={'size': 12})

# Set size of figure
fig.set_size_inches(7.2, 5)
plt.savefig('report-plots/svg/compare-filters.svg', dpi=1000, bbox_inches='tight')
plt.show()
