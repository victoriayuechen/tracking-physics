import sys
import pandas as pd
import matplotlib.pyplot as plt 
import numpy as np

tracking_time_1 = pd.read_csv('results/timing-ds-1.txt', delimiter=',').to_numpy() 
tracking_time_2 = pd.read_csv('results/timing-ds-2.txt', delimiter=',').to_numpy() 
tracking_time_3 = pd.read_csv('results/timing-ds-3.txt', delimiter=',').to_numpy() 
tracking_time_4 = pd.read_csv('results/timing-ds-4.txt', delimiter=',').to_numpy() 
tracking_time_5 = pd.read_csv('results/timing-ds-5.txt', delimiter=',').to_numpy() 
actual_time = np.ones(len(tracking_time_1)) * (1 / 60)
ds_level = [0.00, 0.01, 0.04, 0.08, 0.10]

times = [tracking_time_1, tracking_time_2, tracking_time_3, tracking_time_4, tracking_time_5]
for i in range(len(ds_level)):
    plot_tracking = np.cumsum((1 / 1000) * times[i])
    plt.plot(range(0, len(plot_tracking)), plot_tracking, label='Tracking Time - Downsample = {ds:.3f}'.format(ds=ds_level[i]))
    plot_tracking[0] = 0 

plot_actual = np.cumsum(actual_time)
plot_actual[0] = 0 

plt.plot(range(0, len(actual_time)), plot_actual, label='Frame Time')

plt.ylabel('Time (s)')
plt.xlabel('Number of frames processed')
plt.title('Comparison with frame time and tracking time')
plt.legend()
plt.savefig('exp-results/timing-ds.png')
plt.show()