import pandas as pd
import matplotlib.pyplot as plt 
import numpy as np

truth = pd.read_csv('truth.txt', delimiter=',').to_numpy()
guess_1000 = pd.read_csv('guess.txt', delimiter=',').to_numpy()

# Euclidean distance between the centroids
dist_1000 = np.linalg.norm((truth - guess_1000), axis=1)
plt.plot(range(0, len(dist_1000)), dist_1000, label='1000 particles')


plt.ylabel('Euclidean distance')
plt.xlabel('Number of frames processed')
plt.title('(Stationary) Euclidean distance between point cloud center and particle cloud center')
plt.legend()
plt.savefig('performance-tracking.png')
plt.show()