import sys
import pandas as pd
import matplotlib.pyplot as plt 
import numpy as np

# Find the experiment type 
experiment_type = sys.argv[1]
#truth = pd.read_csv('results/translation_cube.txt', delimiter=',').to_numpy()
generated = pd.read_csv('results/static.txt', delimiter=',').to_numpy()
guess_1000 = pd.read_csv('results/static-guess.txt', delimiter=',').to_numpy()

show_norm = True
show_x = True
show_y = True
show_z = True

#x_actual = truth[:, 0]
x_guess = guess_1000[:, 0]
x_gen = generated[:, 0]

#y_actual = truth[:, 1]
y_guess = guess_1000[:, 1]
y_gen = generated[:, 1]

#z_actual = truth[:, 2]
z_guess = guess_1000[:, 2]
z_gen = generated[:, 2]

if show_norm:
    fig1 = plt.figure(1)

    # Euclidean distance between the centroids
    dist_1000 = np.linalg.norm((generated - guess_1000), axis=1)
    plt.plot(range(0, len(dist_1000)), dist_1000, label='700 particles')

    plt.xlim(0)
    plt.ylim(0)
    plt.ylabel('L2 Norm')
    plt.xlabel('Number of frames processed')
    plt.title('L2 norm between actual centroid and predicted centroid')
    plt.legend()
    plt.savefig('plots/distance-{experiment}.pdf'.format(experiment=experiment_type))

if show_x:
    figx = plt.figure(2)

    #plt.plot(range(0, len(dist_1000)), x_actual, 'r', label="Actual movement over x-axis")
    plt.plot(range(0, len(dist_1000)), x_guess, 'b', label="Predicted movement over x-axis")
    plt.plot(range(0, len(dist_1000)), x_gen, 'g', label="generated movement over x-axis", linestyle='dashed', linewidth='2')

    plt.xlim(0)
    #plt.ylim(0)
    plt.ylabel('Movement about x-axis')
    plt.xlabel('Number of frames processed')
    plt.title('Movement about x-axis over number of frames processed')
    plt.legend()
    plt.savefig('plots/movement-x-{experiment}.pdf'.format(experiment=experiment_type))

if show_y:
    figy = plt.figure(3)


    #plt.plot(range(0, len(dist_1000)), y_actual, 'r', zorder=10, label="Actual movement over y-axis")
    plt.plot(range(0, len(dist_1000)), y_guess, 'b', zorder=-1, label="Predicted movement over y-axis")
    plt.plot(range(0, len(dist_1000)), y_gen, 'g', label="generated movement over x-axis", linestyle='dashed', linewidth='2')

    plt.xlim(0)
    #plt.ylim(0)
    plt.ylabel('Movement about y-axis')
    plt.xlabel('Number of frames processed')
    plt.title('Movement about y-axis over number of frames processed')
    plt.legend()
    plt.savefig('plots/movement-y-{experiment}.pdf'.format(experiment=experiment_type))


if show_z:
    figz = plt.figure(4)

    #plt.plot(range(0, len(dist_1000)), z_actual, 'r', label="Actual movement over z-axis")
    plt.plot(range(0, len(dist_1000)), z_guess, 'b', label="Predicted movement over z-axis")
    plt.plot(range(0, len(dist_1000)), z_gen, 'g', label="generated movement over x-axis", linestyle='dashed', linewidth='2')

    plt.xlim(0)
    #plt.ylim(0)
    plt.ylabel('Movement about z-axis')
    plt.xlabel('Number of frames processed')
    plt.title('Movement about z-axis over number of frames processed')
    plt.legend()
    plt.savefig('plots/movement-z-{experiment}.pdf'.format(experiment=experiment_type))

plt.show()
