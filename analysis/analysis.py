import sys
import pandas as pd
import matplotlib.pyplot as plt 
import numpy as np

# Find the experiment type 
experiment_type = sys.argv[1]
truth = pd.read_csv('results/truth-monke.txt', delimiter=',').to_numpy()
guess_1000 = pd.read_csv('results/guess-monke.txt', delimiter=',').to_numpy()

# Euclidean distance between the centroids
dist_1000 = np.linalg.norm((truth - guess_1000), axis=1)
plt.plot(range(0, len(dist_1000)), dist_1000, label='700 particles')

x_actual = truth[:, 0]
x_guess = guess_1000[:, 0]

fig1 = plt.figure(1)

plt.xlim(0)
plt.ylim(0)
plt.ylabel('L2 Norm')
plt.xlabel('Number of frames processed')
plt.title('L2 norm between actual centroid and predicted centroid')
plt.legend()
plt.savefig('distance-{experiment}.png'.format(experiment=experiment_type))

fig2 = plt.figure(2)

plt.plot(range(0, len(dist_1000)), x_actual, 'r', label="Actual movement over x-axis")
plt.plot(range(0, len(dist_1000)), x_guess, 'b', label="Predicted movement over x-axis")

plt.xlim(0)
plt.ylim(0)
plt.ylabel('Movement about x-axis')
plt.xlabel('Number of frames processed')
plt.title('Movement about x-axis over number of frames processed')
plt.legend()
plt.savefig('movement-{experiment}.png'.format(experiment=experiment_type))

plt.show()


plt.show()