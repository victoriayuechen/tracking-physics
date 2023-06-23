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

res1_time = []
res1_accuracy = []

res2_time = []
res2_accuracy = []

res3_time = []
res3_accuracy = []

res4_time = []
res4_accuracy = []

res5_time = []
res5_accuracy = []

true = pd.read_csv('experiment-full/true-translation.txt', delimiter=',').to_numpy()[:, 0]
accuracies = [res1_accuracy, res2_accuracy, res3_accuracy, res4_accuracy, res5_accuracy]
times = [res1_time, res2_time, res3_time, res4_time, res5_time]

# For each resolution of the camera 
for i in range(1, 6):
    # For all the particle counts 
    for j in range(1, 6):
        res = pd.read_csv('experiment-full/res{resN}-{num}.txt'.format(resN=i, num=j), delimiter=',').to_numpy()[:, 0]
        res = np.linalg.norm(res - true[:len(res)])
        accuracies[i - 1].append(res)

# For each resolution of the camera 
for i in range(1, 6):
    # For all the particle counts 
    for j in range(1, 6):
        res = pd.read_csv('experiment-full/res{resN}-timing-{num}.txt'.format(resN=i, num=j), delimiter=',').to_numpy()[:, 0]
        times[i - 1].append(np.mean(res))

plt.plot(res1_accuracy, res1_time, label="30 x 40 Camera", marker='o', c='royalblue')
plt.plot(res2_accuracy, res2_time, label="40 x 50 Camera", marker='o', c='darkorange')
plt.plot(res3_accuracy, res3_time, label="50 x 60 Camera", marker='o', c='crimson')
plt.plot(res4_accuracy, res4_time, label="60 x 70 Camera", marker='o', c='limegreen')
plt.plot(res5_accuracy, res5_time, label="70 x 80 Camera", marker='o', c='violet')

plt.ylabel('Average L2 Norm Error')
plt.xlabel('Average Frame Processing Time [ms]')
plt.xlim(left=0)
plt.ylim(bottom=0)
plt.title('Pareto Frontiers')
plt.legend(loc=2, prop={'size': 12})

# Save the file 
fig.set_size_inches(7.2, 5)
plt.savefig('report-plots/test.svg', dpi=1000, bbox_inches='tight')
plt.show()
