import numpy as np
import math
import matplotlib.pyplot as plt
import time


def plot_icp(X, P, P0, i, rmse):
    plt.cla()
    plt.scatter(X[0, :], X[1, :], c='k', marker='o', s=50, lw=0)
    plt.scatter(P[0, :], P[1, :], c='r', marker='o', s=50, lw=0)
    plt.scatter(P0[0, :], P0[1, :], c='b', marker='o', s=50, lw=0)
    plt.legend(('X', 'P', 'P0'), loc='lower left')
    plt.plot(np.vstack((X[0, :], P[0, :])),
             np.vstack((X[1, :], P[1, :])), c='k')
    plt.title("Iteration: " + str(i) + "  RMSE: " + str(rmse))
    plt.axis([-10, 15, -10, 15])
    plt.gca().set_aspect('equal', adjustable='box')
    plt.draw()
    plt.savefig('images/icp_a' + str(i) + '.png')
    plt.pause(0.5)
    return


def generate_data():

    # create reference data
    X = np.array([[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 9, 9, 9, 9],
                  [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, -1, -2, -3, -4, -5]])

    # add noise
    P = X + 0.05 * np.random.normal(0, 1, X.shape)

    # translate
    P[0, :] = P[0, :] + 1
    P[1, :] = P[1, :] + 1

    # rotate
    theta1 = np.radians(10.0)
    theta2 = np.radians(110.0)
    rot1 = np.array([[math.cos(theta1), -math.sin(theta1)],
                     [math.sin(theta1),  math.cos(theta1)]])
    rot2 = np.array([[math.cos(theta2), -math.sin(theta2)],
                     [math.sin(theta2),  math.cos(theta2)]])

    # sets with known correspondences
    P1 = np.dot(rot1, P)
    P2 = np.dot(rot2, P)

    # sets with unknown correspondences
    P3 = np.random.permutation(P1.T).T
    P4 = np.random.permutation(P2.T).T

    return X, P1, P2, P3, P4
