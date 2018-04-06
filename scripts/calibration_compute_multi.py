#!/usr/bin/env python
# -*- coding: utf-8 -*-

# calibration_compute_multi.py: Code to compute absolute orientation from collected points
# Author: Nishanth Koganti, Ravi Joshi
# Date: 2016/06/16
# Source: http://math.stackexchange.com/questions/745234/calculate-rotation-translation-matrix-to-match-measurement-points-to-nominal-poi

# import modules
import yaml
import rospy
import datetime
import matplotlib
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from tf.transformations import euler_from_matrix, quaternion_from_euler

# matplotlib settings
matplotlib.rcParams.update({'font.size': 12})


def absOrientation(x, y):
    # number of samples
    nSamples = x.shape[0]

    # center data
    xMean = x.mean(axis=0)
    yMean = y.mean(axis=0)

    xTemp = x - xMean
    yTemp = y - yMean

    # get the variance
    xSD = np.mean(np.sum(xTemp**2, 1))
    np.mean(np.sum(yTemp**2, 1))

    # get covariance matrix
    covarMatrix = np.dot(yTemp.T, xTemp) / nSamples

    # apply singular value decomposition
    U, D, V = np.linalg.svd(covarMatrix, full_matrices=True, compute_uv=True)
    V = V.T.copy()

    S = np.diag(np.asarray([1, 1, np.sign(np.linalg.det(V) * np.linalg.det(U))]))

    # get scaling factor
    c = np.trace(np.dot(np.diag(D), S)) / xSD

    # get rotation matrix
    R = c * np.dot(np.dot(U, S), V.T)

    # get translation vector
    t = yMean - np.dot(R, xMean)

    # compute transformation error
    xOut = (np.dot(R, x.T)).T + t
    errs = np.sqrt(np.sum((y - xOut)**2, axis=1))
    err  = errs.sum() / nSamples
    return xOut, R, t, err

def main():
    # initialize ros node
    rospy.init_node('compute_calibration', anonymous=True)

    # get path to folder containing recorded data
    filesPath = rospy.get_param('~data_dir')

    # plot both point sets together
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    baxterFile = filesPath + 'position_wrt_baxter_multiple_kinect.csv'
    baxterTraj = np.loadtxt(baxterFile, delimiter=',', skiprows=1)
    ax.scatter(baxterTraj[:, 0], baxterTraj[:, 1], baxterTraj[:, 2], label='Baxter')

    for i in range(1, 4):
        kinectFile = filesPath + 'position_wrt_multiple_kinect_' + str(i) + '.csv'

        rospy.loginfo('Reading files %s and %s' % (kinectFile, baxterFile))

        # load trajectories
        kinectTraj = np.loadtxt(kinectFile, delimiter=',', skiprows=1)

        # compute absolute orientation
        kinectOut, rot, trans, err = absOrientation(kinectTraj, baxterTraj)

        # output results
        err_cm = err * 100.0
        rospy.loginfo('Calibration Error: %.2f cm' % err_cm)

        # get rotation matrix as quaternion and euler angles
        euler = euler_from_matrix(rot)
        quat = quaternion_from_euler(*euler)

        # save results to yaml file
        calibration = {'parent'    : 'base',
                       'child'     : 'kinect' + str(i) + '_link',
                       'trans'     : trans.tolist(),
                       'rot'       : quat.tolist(),
                       'rot_euler' : list(euler),
                       'calibration error (m)' : float('%.4f' % err),
                       'created on' : datetime.datetime.now().strftime('%d %B %Y %I:%M:%S %p')}

        with open('%sbaxter_kinect_%s_calibration.yaml' % (filesPath, i), 'w') as outfile:
            yaml.dump(calibration, outfile)

        # plot the data
        ax.scatter(kinectOut[:, 0], kinectOut[:, 1], kinectOut[:, 2], label='Kinect %s (err: %.2f cm)' % (i, err_cm))

    # add labels and title
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.legend()
    ax.grid(True)
    ax.set_title('Multiple Kinect-Baxter Calibration')
    plt.show()

if __name__ == '__main__':
    main()
