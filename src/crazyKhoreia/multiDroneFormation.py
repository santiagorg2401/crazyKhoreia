#!/usr/bin/env python3

import os
from itertools import combinations

import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import KMeans

from crazyKhoreia._3DIoU import _3DIoU
from crazyKhoreia.crazyKhoreia import crazyKhoreia


class multiDroneFormation(crazyKhoreia):
    """
    CrazyKhoreia's child class multiDroneFormation computes a feasible swarm formation.
    Attributes:
        dims        (array):    2x3 float array containing flight space constraints in the x, y, and z axis [[MIN_X, MIN_Y, MIN_Z],[MAX_X, MAX_Y, MAX_Z]]
        boxShape    (list):     1x3 float list with the aerodynamical downwash effect constraints along the x, y and z axis.
        in_path     (str):      Global image's path to process.
        out_path    (str):      File output path.
        nmbr_drones (int):      Number of UAVs in swarm.

        led         (bool):     Whether or not to control LED light relative to out of contour travel. TODO: Is this really necessary?
        cnt_scaled  (list):     List of processed contours.
        positions   (list):     List of formation positions.

    Methods:
        get_clusters(wayPoints):
            Get a cluster centroids array from a waypoint matrix, the number of clusters equals the number of UAVs in swarm.
        get_idealPositions(cc):
            Obtain the ideal positions from the cluster centroids and visualize them.
        getIoUsppd():
            An iterative cycle that evaluates the Intersection over the Union of a pair of UAVs and correct their position along the perpendicular axis to avoid inter-drone collisions.
        save():
            Export the positions in a .csv file.

    """

    def __init__(self, dims, boxShape, in_path, out_path, nmbr_drones):
        super().__init__(dims, in_path, led=False)

        self.dims, self.boxShape, self.in_path, self.out_path = dims, boxShape, in_path, out_path
        self.nmbr_drones = nmbr_drones

        wayPoints = self.get_waypoints()
        self.plot_contour_inspection(wayPoints)

        cc = self.get_clusters(wayPoints)
        self.positions = self.get_idealPositions(cc)
        self.getIoUsppd()

        self.save()

        plt.show()

    def get_clusters(self, wayPoints):
        fig, ax0 = plt.subplots()
        kmeans = KMeans(n_clusters=self.nmbr_drones, random_state=0)
        y_pred = kmeans.fit_predict(
            np.array([wayPoints[:, 1], wayPoints[:, 2]]).T)
        cc = kmeans.cluster_centers_

        ax0.scatter(wayPoints[:, 1], wayPoints[:, 2], c=y_pred)
        ax0.plot(cc[:, 0], cc[:, 1], 'o', c='violet',
                 label='Cluster centroids.')
        ax0.legend()
        ax0.set_title("KMeans clusters.")

        return cc

    def get_idealPositions(self, cc):
        positions = np.empty(shape=(0, 3))

        positions = np.array(
            [(self.dims[0][2])*np.ones(shape=(len(cc),)), cc[:, 0], cc[:, 1]]).T

        xs = positions[:, 0]
        ys = positions[:, 1]
        zs = positions[:, 2]

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        ax.scatter(xs, ys, zs)
        ax.set_title("Ideal UAV formation inspection.")

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        return positions

    def getIoUsppd(self):
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        xs = self.positions[:, 0]
        ys = self.positions[:, 1]
        zs = self.positions[:, 2]

        ax.scatter(xs, ys, zs, marker='D', label="Ideal goal positions.")
        for i in range(0, self.nmbr_drones):
            ax.text(xs[i], ys[i], zs[i], str(i), ha='left', c='b')

        while(1):
            print("Minimizing IoU ...")
            combs = combinations(range(0, self.nmbr_drones), 2)
            UAV_IoU = np.zeros(shape=(self.nmbr_drones, 1))

            for i in combs:
                box1 = [self.boxShape, 0,
                        self.positions[i[0]]/np.array([1, 1, 2])]
                box2 = [self.boxShape, 0,
                        self.positions[i[1]]/np.array([1, 1, 2])]
                IoU = _3DIoU(box1, box2)

                UAV_IoU[i[0]] += IoU.IOU_3d
                UAV_IoU[i[1]] += IoU.IOU_3d

            maxIoU = max(UAV_IoU)
            maxUAV = np.where(UAV_IoU == maxIoU)[0][0]
            print("Maximum IoU: " + str(maxIoU))

            if maxIoU != 0:
                if max(self.positions[:, 0]) <= self.dims[1][2]:
                    self.positions[maxUAV][0] += self.boxShape[0]
                else:
                    print("Formation for " + str(self.nmbr_drones) + " UAVs failed at UAV " + str(
                        maxUAV) + " depth limitations exceeded, please lower the number of UAVs.")
                    break
            elif maxIoU == 0:
                print("Interception free formation found.")
                break

        xs = self.positions[:, 0]
        ys = self.positions[:, 1]
        zs = self.positions[:, 2]

        ax.scatter(xs, ys, zs, marker='s', label="Adjusted goal positions.")
        for i in range(0, self.nmbr_drones):
            ax.text(xs[i], ys[i], zs[i], str(i), ha='left', c='orange')
        ax.legend()

    def save(self):
        file_name = os.path.basename(self.in_path)
        name = file_name.split('.', 1)[0]

        np.savetxt(self.out_path + name +
                   '_mdf_pos.csv', self.positions, delimiter=",")
