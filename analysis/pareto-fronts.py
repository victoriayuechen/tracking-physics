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

# Get the true positions
true = pd.read_csv('experiment-full/true-translation.txt', delimiter=',').to_numpy()[:, 0]

# Gets the accuracy line for a camera with resolution `res`
def get_accuracy(res):
    res_acc = np.zeros((6, 5))

    # For each runs  
    for i in range(1, 6):
        # For all the particle counts 
        for j in range(1, 6):
            predictions = pd.read_csv('experiment-full/runs/{runCount}-res{resN}-{num}.txt'.format(runCount=i, resN=res, num=j), delimiter=',').to_numpy()[:, 0]
            res_acc[i, j - 1] = np.mean(np.abs(predictions - true[:len(predictions)]))
    
    # From the run without the correct names 
    for j in range(1, 6):
        predictions = pd.read_csv('experiment-full/res{resN}-{num}.txt'.format(resN=res, num=j), delimiter=',').to_numpy()[:, 0]
        res_acc[0, j - 1] = np.mean(np.abs(predictions - true[:len(predictions)]))
    
    return np.mean(res_acc, axis=0)


# Gets the accuracy line for a camera with resolution `res`
def get_times(res):
    res_time = np.zeros((6, 5))
    print(res)
    # For each runs  
    for i in range(1, 6):
        # For all the particle counts 
        for j in range(1, 6):
            print("{i1} {j1}".format(i1=i, j1=j))
            fileName = 'experiment-full/runs/{runCount}-res{resN}-timing-{num}.txt'.format(runCount=i, resN=res, num=j)
            times = pd.read_csv(fileName, delimiter=',').to_numpy()
            res_time[i, j - 1] = np.mean(times)
    
    # From the run without the correct names 
    for j in range(1, 6):
        times = pd.read_csv('experiment-full/res{resN}-timing-{num}.txt'.format(resN=res, num=j), delimiter=',').to_numpy()
        res_time[0, j - 1] = np.mean(times)
    
    return np.mean(res_time, axis=0)

labels = ["30 x 40 Camera", "40 x 50 Camera", "50 x 60 Camera", "60 x 70 Camera", "70 x 80 Camera"]
colors = ['royalblue', 'darkorange', 'crimson', 'limegreen', 'violet']

for i in range(1, 6):
    xs_time = get_times(i)
    ys_accuracy = get_accuracy(i)
    plt.plot(xs_time, ys_accuracy, label=labels[i - 1], marker='o', c=colors[i - 1])

plt.ylabel('Average L2 Norm Error')
plt.xlabel('Average Frame Processing Time [ms]')
plt.xlim(left=0)
plt.ylim(bottom=0)
plt.title('Pareto Frontiers')
plt.legend(loc=2, prop={'size': 12})

# Save the file 
fig.set_size_inches(7.2, 5)
plt.savefig('report-plots/test-2.svg', dpi=1000, bbox_inches='tight')
plt.show()
