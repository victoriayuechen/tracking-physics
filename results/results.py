import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

time_test = pd.read_csv("timetest.csv", delimiter=',')

particles = time_test['PARTICLES']
setup = time_test['SETUP']
tracking = time_test['TRACKING']
total = time_test['TOTAL']
frame = time_test['FRAME']

fig1 = plt.figure(1)

plt.plot(particles, setup, 'r', lw=3, label= "Setup time")
plt.xlabel("Number of particles")
plt.ylabel("Setup time (in ms)")
plt.title("Setup time based on number of particles")

fig2 = plt.figure(2)

plt.plot(particles, tracking, 'r', lw=3, label= "Tracking time")
plt.xlabel("Number of particles")
plt.ylabel("Tracking time (in ms)")
plt.title("Tracking time based on number of particles")

fig3 = plt.figure(3)

plt.plot(particles, tracking, 'r', lw=3, label= "Total time")
plt.xlabel("Number of particles")
plt.ylabel("Total time (in ms)")
plt.title("Total time based on number of particles")

fig4 = plt.figure(4)
plt.plot(particles, frame, 'r', lw=3, label= "Time per frame")
plt.xlabel("Number of particles")
plt.ylabel("Time per frame (in ms)")
plt.title("Time per frame based on number of particles")

plt.show()

# old_data = pd.read_csv('csv1.csv', delimiter=',')
# new_data = pd.read_csv('csv2.csv', delimiter=',')

# particles = old_data['PARTICLES']
# old_error = old_data['ERROR']
# old_time = old_data['TIME']
# new_error = new_data['ERROR']
# new_time = new_data['TIME']

# fig1 = plt.figure(1)

# plt.plot(particles, old_error, 'r', lw=3, label= "Error in current system")
# plt.plot(particles, new_error, 'b', lw=3, label= "Error in GPU accelerated system")
# plt.xlabel("Number of particles")
# plt.ylabel("RMS Error")
# plt.title("RMS Error of tracking system (x - axis)")
# plt.legend()

# fig2 = plt.figure(2)

# plt.plot(particles, old_time, 'r', lw=3, label= "Execution time in current system")
# plt.plot(particles, new_time, 'b', lw=3, label= "Execution time in GPU accelerated system")
# plt.xlabel("Number of particles")
# plt.ylabel("Time (in ms)")
# plt.title("Exectution time of tracking system")
# plt.legend()

# fig3 = plt.figure(3)

# plt.plot(old_error, old_time, 'r', lw=3, label= "OLD_TIME_ERROR")
# plt.plot(new_error, new_time, 'b', lw=3, label= "NEW_TIME_ERROR")
# plt.xlabel("Error")
# plt.ylabel("Time")
# plt.title("Error and time of tracking system")
# plt.legend()
# plt.show()