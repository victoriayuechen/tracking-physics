import sys
import pandas as pd
import matplotlib.pyplot as plt 
import numpy as np

tracking_time_1000 = pd.read_csv('results/timing-1000.txt', delimiter=',').to_numpy() 
tracking_time_500 = pd.read_csv('results/timing-500.txt', delimiter=',').to_numpy() 
tracking_time_250 = pd.read_csv('results/timing-250.txt', delimiter=',').to_numpy() 
actual_time = np.ones(len(tracking_time_250)) * (1 / 50)

plot_tracking_1000 = np.cumsum((1 / 1000) * tracking_time_1000)
plot_tracking_500 = np.cumsum((1 / 1000) * tracking_time_500)
plot_tracking_250 = np.cumsum((1 / 1000) * tracking_time_250)
plot_actual = np.cumsum(actual_time)

plot_tracking_1000[0] = 0 
plot_tracking_500[0] = 0 
plot_tracking_250[0] = 0 
plot_actual[0] = 0

plt.plot(range(0, len(tracking_time_1000)), plot_tracking_1000, label='Tracking Time (1000 particles)')
plt.plot(range(0, len(tracking_time_500)), plot_tracking_500, label='Tracking Time (500 particles)')
plt.plot(range(0, len(tracking_time_250)), plot_tracking_250, label='Tracking Time (250 particles)')
plt.plot(range(0, len(actual_time)), plot_actual, label='Frame Time')


plt.ylabel('Time (s)')
plt.xlabel('Number of frames processed')
plt.title('Comparison with frame time and tracking time')
plt.legend()
plt.savefig('exp-results/timing.png')
plt.show()