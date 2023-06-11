import sys
import pandas as pd
import matplotlib.pyplot as plt 
import numpy as np

# Find the experiment type 
experiment_type = sys.argv[1]
truth = pd.read_csv('experiment-full/true-translation.txt', delimiter=',').to_numpy()
guess_1 = pd.read_csv('experiment-full/translation-2.txt', delimiter=',').to_numpy()

delta_x_actual = truth[:, 2]
delta_x_pred1 = guess_1[:, 2]

plt.plot(range(len(truth)), delta_x_actual, label='Actual z Movement', linewidth=2.5)
plt.plot(range(len(delta_x_pred1)), delta_x_pred1, label='Predicted z Movement', alpha=0.5, linestyle='--')
plt.xlim(left=0)
plt.ylabel('Z-position')
plt.xlabel('Number of frames processed')
plt.title('Movement along Z-axis')
plt.legend()
plt.savefig('z-pos-{experiment}.png'.format(experiment=experiment_type))
plt.show()