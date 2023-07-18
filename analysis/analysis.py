import sys
import math
import pandas as pd
import matplotlib.pyplot as plt 
import numpy as np

# Find the experiment type 
experiment_type = sys.argv[1]

# Font size and .tex background
params = {'text.usetex' : True,
          'font.size' : 15,
          'font.family' : 'lmodern'
          }
plt.rcParams.update(params) 

# particle_amount = [50, 100, 200, 500, 1000, 2000]
# rmse_6dof = [0.20410247458350628, 0.1428133792552336, 0.12165691828482854, 0.10888112757752207, 0.07931401945040527, 0.056359782697426974]
# rmse_1dof = [0.0302533276907476, 0.030035196636135877, 0.028963000204013996, 0.02954770807536555, 0.029552348516970327, 0.029694688859071578]

# plt.plot(particle_amount, rmse_6dof, 'r', label="6 DoF", marker="x")
# plt.plot(particle_amount, rmse_1dof, 'b', label="2 DoF", marker="x")

# plt.xlim(0)
# plt.ylim(0)
# plt.ylabel('RMSE over 500 frames (in metres)')
# plt.xlabel('Amount of particles used')
# plt.title('RMSE of a particle filter when resampling for 6 DoF and 2 DoF')
# plt.legend(loc=1, prop={'size': 12})
# plt.savefig('plots/actualresults/results-{experiment}.pdf'.format(experiment=experiment_type), dpi=1000, bbox_inches='tight')

# plt.show()

truth = pd.read_csv('results/translating_cube_2dof.txt', delimiter=',').to_numpy()
#generated = pd.read_csv('results/static-cube-6dof.txt', delimiter=',').to_numpy()
guess_1000 = pd.read_csv('results/translating-6dof-guess', delimiter=',').to_numpy()
guess_1dof = pd.read_csv('results/translating-1dof-guess', delimiter=',').to_numpy()

# show_norm = True
show_x = True
show_y = True
show_z = True

x_actual = truth[:, 0]
# x_guess = guess_1000[:, 0]
# #x_gen = generated[:, 0]

y_actual = truth[:, 1]
y_guess = guess_1000[:, 1]
#y_gen = generated[:, 1]

z_actual = truth[:, 2]
# z_guess = guess_1000[:, 2]
# #z_gen = generated[:, 2]

# x_1dof = guess_1dof[:, 0]
y_1dof = guess_1dof[:, 1]
# z_1dof = guess_1dof[:, 2]


# if show_norm:
#     fig1 = plt.figure(1)

#     # Euclidean distance between the centroids
dist_1000 = np.linalg.norm((truth - guess_1000), axis=1)
dist_1dof = np.linalg.norm((truth - guess_1dof), axis=1)

#     plt.plot(range(0, len(dist_1000)), dist_1000, 'r', label='6 DoF')
#     plt.plot(range(0, len(dist_1dof)), dist_1dof, 'b', zorder=10, label='1 DoF')

#     plt.xlim(0)
#     plt.ylim(0)
#     plt.ylabel('L2 Norm (in metres)')
#     plt.xlabel('Number of frames processed')
#     plt.title('L2 norm between actual centroid and predicted centroid')
#     plt.legend()
#     plt.savefig('plots/actualresults/distance-{experiment}.pdf'.format(experiment=experiment_type))

if show_x:
    figx = plt.figure(2)

    plt.plot(range(0, 499), x_actual, 'r', zorder=10, label="Movement of the cube")
    # plt.plot(range(0, len(dist_1000)), x_guess, 'b', zorder=-1, label="Predicted movement with 6 DoF")
    # plt.plot(range(0, len(dist_1000)), x_1dof, 'k', zorder=5, label="Predicted movement with 2 DoF")
    #plt.plot(range(0, len(dist_1000)), x_gen, 'g', label="generated movement over x-axis", linestyle='dashed', linewidth='2')

    plt.xlim(0)
    #plt.ylim(0)
    plt.ylabel('Movement about x-axis (in metres)')
    plt.xlabel('Number of frames processed')
    plt.title('Movement of cube about x-axis over number of frames processed')
    # plt.legend()
    plt.savefig('plots/actualresults/movement-x-{experiment}.pdf'.format(experiment=experiment_type), dpi=1000, bbox_inches='tight')

if show_y:
    figy = plt.figure(3)


    plt.plot(range(0, len(dist_1000)), y_actual, 'r', zorder=10, label="Actual movement", linestyle='dashed')
    plt.plot(range(0, len(dist_1000)), y_guess, 'b', zorder=-1, label="Predicted movement with 6 DoF")
    plt.plot(range(0, len(dist_1000)), y_1dof, 'k', zorder=5, label="Predicted movement with 2 DoF")
    #plt.plot(range(0, len(dist_1000)), y_gen, 'g', label="generated movement over x-axis", linestyle='dashed', linewidth='2')

    plt.xlim(0)
    #plt.ylim(0)
    plt.ylabel('Movement about y-axis (in metres)')
    plt.xlabel('Number of frames processed')
    plt.title('Movement about y-axis over number of frames processed')
    plt.legend(loc=1, prop={'size': 9})
    plt.savefig('plots/actualresults/movement-y-{experiment}.pdf'.format(experiment=experiment_type), dpi=1000, bbox_inches='tight')


if show_z:
    figz = plt.figure(4)

    plt.plot(range(0, 499), z_actual, 'r', zorder=10, label="Movement of the cube")
    # plt.plot(range(0, len(dist_1000)), z_guess, 'b', zorder=-1, label="Predicted movement with 6 DoF")
    # plt.plot(range(0, len(dist_1000)), z_1dof, 'k', zorder=-5, label="Predicted movement with 2 DoF")
    #plt.plot(range(0, len(dist_1000)), z_gen, 'g', label="generated movement over x-axis", linestyle='dashed', linewidth='2')

    plt.xlim(0)
    #plt.ylim(0)
    plt.ylabel('Movement about z-axis (in metres)')
    plt.xlabel('Number of frames processed')
    plt.title('Movement of the cube about z-axis over number of frames processed')
    # plt.legend()
    plt.savefig('plots/actualresults/movement-z-{experiment}.pdf'.format(experiment=experiment_type), dpi=1000, bbox_inches='tight')

    plt.show()

# # plt.show()

# mse_6dof = np.square(np.subtract(truth,guess_1000)).mean() 
# rmse_6dof = math.sqrt(mse_6dof)
# print("6 DoF RMSE: " + str(rmse_6dof) + "\n")

# mse_1dof = np.square(np.subtract(truth,guess_1dof)).mean() 
# rmse_1dof = math.sqrt(mse_1dof)
# print("1 DoF RMSE: " + str(rmse_1dof) + "\n")