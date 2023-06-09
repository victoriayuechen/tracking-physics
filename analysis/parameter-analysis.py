import sys
import pandas as pd
import matplotlib.pyplot as plt 
import numpy as np

# Find the experiment type 
experiment_type = sys.argv[1]

truth = pd.read_csv('results/truth-exp4-suzane.txt', delimiter=',').to_numpy()
param1 = pd.read_csv('results/guess-exp4-suzanne-p1.txt', delimiter=',').to_numpy()
param2 = pd.read_csv('results/guess-exp4-suzanne-p2.txt', delimiter=',').to_numpy()
param3 = pd.read_csv('results/guess-exp4-suzanne-p3.txt', delimiter=',').to_numpy()
param4 = pd.read_csv('results/guess-exp4-suzanne-p4.txt', delimiter=',').to_numpy()
param5 = pd.read_csv('results/guess-exp4-suzanne-p5.txt', delimiter=',').to_numpy()

data = [param1, param2, param3, param4, param5]
parameters = [0.0, 0.01, 0.04, 0.08, 0.1]

for i in range(len(data)):
    diff = np.linalg.norm((truth - data[i]), axis=1)
    plt.plot(range(0, len(diff)), diff, label='{} = {}'.format(experiment_type, parameters[i]))

plt.ylabel('L2 Norm')
plt.xlabel('Number of frames processed')
plt.title('L2 norm between actual centroid and predicted centroid')
plt.legend()
plt.savefig('exp-results/param-performance-suzanne-{experiment}.png'.format(experiment=experiment_type))
plt.show()
