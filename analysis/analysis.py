import sys
import pandas as pd
import matplotlib.pyplot as plt 
import numpy as np

# Find the experiment type 
experiment_type = sys.argv[1]
truth = pd.read_csv('full-model/true-pos.txt', delimiter=',').to_numpy()
guess_1 = pd.read_csv('network-exp/guess-1.txt', delimiter=',').to_numpy()
guess_2 = pd.read_csv('network-exp/guess-1-playback.txt', delimiter=',').to_numpy()

# # Euclidean distance between the centroids
# dist_1000 = np.linalg.norm((truth - guess_1000), axis=1)
# plt.plot(range(0, len(dist_1000)), dist_1000, label='1000 particles')

# plt.ylabel('L2 Norm')
# plt.xlabel('Number of frames processed')
# plt.title('L2 norm between actual centroid and predicted centroid')
# plt.legend()
# plt.savefig('plots/performance-{experiment}.png'.format(experiment=experiment_type))
# plt.show()

delta_x_actual = truth[:, 0]
delta_x_pred1 = guess_1[:, 0]
delta_x_pred2 = guess_2[:, 0]

plt.plot(range(len(truth)), delta_x_actual, label='Actual x Movement')
plt.plot(range(len(delta_x_pred1)), delta_x_pred1, label='Predicted x Movement (Real time)')
plt.plot(range(len(delta_x_pred2)), delta_x_pred2, label='Predicted x Movement (Play back)')

plt.xlim(left=0)
plt.ylim(bottom=0)
plt.ylabel('X-position')
plt.xlabel('Number of frames processed')
plt.title('Movement along X-axis')
plt.legend()
plt.savefig('full-model/x-pos-{experiment}.png'.format(experiment=experiment_type))
plt.show()