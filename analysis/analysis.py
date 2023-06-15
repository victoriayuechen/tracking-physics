import sys
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('pgf')
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

# Find the experiment type 
experiment_type = sys.argv[1]
regular = pd.read_csv('experiment-full/translation-222.txt', delimiter=',').to_numpy()[:, 0]
kld = pd.read_csv('experiment-full/translation-2.txt', delimiter=',').to_numpy()[:, 0]
true = pd.read_csv('experiment-full/true-translation.txt', delimiter=',').to_numpy()[:, 0]

kld = kld[:len(regular)]
true = true[:len(regular)]
xs = np.linspace(0, 6, len(kld))

plt.plot(xs, regular, label='Regular Particle Filter', c='royalblue')
plt.plot(xs, kld, label='KLD Adaptive Filter', c='darkorange')
plt.plot(xs, true, label='Ground Truth', c='k', linewidth=2.5)

plt.xlim(left=0)
plt.ylabel('X-position')
plt.xlabel('Time [s]')
plt.title('Comparison Between Particle Filters')
plt.legend()
plt.savefig('report-plots/compare-filter.pgf', format='pgf')
plt.savefig('report-plots/svg/compare-filter.svg')
plt.show()