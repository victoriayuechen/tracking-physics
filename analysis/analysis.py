import sys
import pandas as pd
import matplotlib.pyplot as plt 
import numpy as np

# Find the experiment type 
experiment_type = sys.argv[1]
truth = pd.read_csv('truth-exp1-suzanne.txt', delimiter=',').to_numpy()
guess_1000 = pd.read_csv('guess-exp1-suzanne.txt', delimiter=',').to_numpy()

# Euclidean distance between the centroids
dist_1000 = np.linalg.norm((truth - guess_1000), axis=1)
plt.plot(range(0, len(dist_1000)), dist_1000, label='1000 particles')

plt.ylabel('L2 Norm')
plt.xlabel('Number of frames processed')
plt.title('L2 norm between actual centroid and predicted centroid')
plt.legend()
plt.savefig('performance-{experiment}.png'.format(experiment=experiment_type))
plt.show()