#!/usr/bin/env python3

import os
from itertools import combinations

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist
from sklearn.cluster import KMeans

from crazyKhoreia._3DIoU import _3DIoU
from crazyKhoreia.crazyKhoreia import crazyKhoreia


class multiDroneFormation(crazyKhoreia):
    """
    CrazyKhoreia's child class multiDroneFormation computes a feasible swarm formation.
    Attributes:
        dims                (array):    2x3 float array containing flight space constraints in the x, y, and z axis [[MIN_X, MIN_Y, MIN_Z],[MAX_X, MAX_Y, MAX_Z]]
        boxShape            (array):    1x3 float array with the aerodynamical downwash effect constraints along the x, y and z axis.
        in_path             (str):      Global image's path to process.
        out_path            (str):      File output path.
        num_drones          (int):      Number of UAVs in swarm.

        led                 (bool):     Whether or not to control LED light relative to out of contour travel. TODO: Is this really necessary?
        cnt_scaled          (array):    Array of processed contours.
        initialGrid         (array):    Array containing the initial drone configuration on ground.
        idealPositions      (array):    Array of ideal formation positions without aerodynamical constraints.
        adjustedPositions   (array):    Array of adjusted positions according to aerodynamical effects.

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

    def __init__(self, dims, boxShape, in_path, out_path, num_drones):
        super().__init__(dims, in_path, led=False)

        self.dims, self.boxShape, self.in_path, self.out_path = np.array(
            dims), np.array(boxShape), in_path, out_path
        self.num_drones = num_drones

        wayPoints = self.get_waypoints()
        self.plot_contour_inspection(wayPoints)

        self.initialGrid = self.estimateInitialGrid()
        cc = self.get_clusters(wayPoints)
        self.idealPositions = self.get_idealPositions(cc)
        self.adjustedPositions = self.getIoUsppd()
        self.centerPositions()
        self.droneAssignments = self.dronePositionAssignment()

        self.visualize()
        self.save()

        plt.show()

    def get_clusters(self, wayPoints):
        fig, ax0 = plt.subplots()
        kmeans = KMeans(
            n_init=10, n_clusters=self.num_drones, random_state=0)
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
        idealPositions = np.empty(shape=(0, 3))

        idealPositions = np.array(
            [(self.dims[0][2])*np.ones(shape=(len(cc),)), cc[:, 0], cc[:, 1]]).T

        xs = idealPositions[:, 0]
        ys = idealPositions[:, 1]
        zs = idealPositions[:, 2]

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        ax.scatter(xs, ys, zs)
        ax.set_title("Ideal UAV formation inspection.")

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        return idealPositions

    def getIoUsppd(self):
        adjustedPositions = np.array(self.idealPositions)

        while(1):
            print("Minimizing IoU ...")
            combs = combinations(range(0, self.num_drones), 2)
            UAV_IoU = np.zeros(shape=(self.num_drones, 1))

            for i in combs:
                box1 = [self.boxShape, 0,
                        adjustedPositions[i[0]]/np.array([1, 1, 2])]
                box2 = [self.boxShape, 0,
                        adjustedPositions[i[1]]/np.array([1, 1, 2])]
                IoU = _3DIoU(box1, box2)

                UAV_IoU[i[0]] += IoU.IOU_3d
                UAV_IoU[i[1]] += IoU.IOU_3d

            maxIoU = max(UAV_IoU)
            maxUAV = np.where(UAV_IoU == maxIoU)[0][0]
            print("Maximum IoU: " + str(maxIoU))

            if maxIoU != 0:
                if max(adjustedPositions[:, 0]) <= self.dims[1][2]:
                    adjustedPositions[maxUAV][0] += self.boxShape[0]
                else:
                    print("Formation for " + str(self.num_drones) + " UAVs failed at UAV " + str(
                        maxUAV) + " depth limitations exceeded, please lower the number of UAVs.")
                    break
            elif maxIoU == 0:
                print("Interception free formation found.")
                break

        return adjustedPositions

    def centerPositions(self):
        centerFS = np.mean(self.dims[:, :2], axis=0)
        maxX = np.max(self.adjustedPositions[:, 0])
        minX = np.min(self.adjustedPositions[:, 0])
        centerSwarm = (maxX - minX)/2.0 + minX
        diff = centerFS - centerSwarm
        self.adjustedPositions[:, 0] += diff[0]

    def estimateInitialGrid(self):
        center = np.mean(self.dims[:, :2], axis=0)
        grid_dim = int(np.ceil(np.sqrt(self.num_drones)))
        x0 = center[0] - (grid_dim/2 - 0.5) * self.boxShape[0]
        y0 = center[1] - (grid_dim/2 - 0.5) * self.boxShape[1]
        initialGrid = np.zeros((self.num_drones, 3))

        for i in range(self.num_drones):
            row = i // grid_dim
            col = i % grid_dim
            initialGrid[i] = [x0 + col * self.boxShape[0],
                              y0 + row * self.boxShape[1], 0]

        return initialGrid

    def dronePositionAssignment(self):
        """
        @article{Nar2022,
            abstract = {Advancements in technology and new drone policies have boosted the adoption of drones by various industries. With active research and development, ingenious applications using drones are being developed. One such innovative and creative application of swarm drones is the drone light show. Light shows are generally performed outdoors in a dedicated clear open air space, which attracts the usage of drones for the execution. A fleet of multiple UAVs illuminates the night sky with picturesque performances in a synchronized manner. Use of drones in light shows provides an eco-friendly and reusable alternative for unique ways of advertisements, celebrations, storytelling, and entertainment. To choreograph the performance of swarm drones means allocating waypoints to each drone for every formation and the complexity of designing increases with the number of drones. An intelligent optimal assignment system is much required to allocate waypoints to drones for each transition in order to create formations. We propose a static swarm-intelligence-based assignment approach named Constrained Hungarian Method for Swarm Drones Assignment (CHungSDA) for optimally assigning multi-UAVs to waypoints. This approach uses Hungarian algorithm as a base with added constraints specific to the targeted application. Several simulations of the proposed approach — carried out for different designs have shown promising results. The work will be of best reference to the UAV manufacturers for their functional design of UAVs and development of Ground Control Station Software to communicate with multiple UAVs, along with other companies that aim to create visual illustrations using UAVs.},
            author = {Dharna Nar and Radhika Kotecha},
            doi = {10.1016/J.RICO.2022.100174},
            issn = {2666-7207},
            journal = {Results in Control and Optimization},
            keywords = {Multi-UAVs,Optimal assignment,Static drone formatics,Swarm drones,Swarm drones light show},
            month = {12},
            pages = {100174},
            publisher = {Elsevier},
            title = {Optimal waypoint assignment for designing drone light show formations},
            volume = {9},
            year = {2022},
        }
        """

        # Compute the pairwise cost matrix between waypoint and drone.
        cost = np.array(cdist(self.initialGrid, self.adjustedPositions))

        row_ind, col_ind = linear_sum_assignment(cost)

        # Print and save the optimal matching.
        print("\nOptimal matching:")
        pos = []
        for i in row_ind:
            print(f"UAV {row_ind[i]} is assigned to waypoint {col_ind[i]}")
            pos.append(self.adjustedPositions[col_ind[i]])

        droneAssignments = np.array(pos)

        return droneAssignments

    def visualize(self):
        # Create and set up plot.
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")

        # Plot ideal goal positions.
        xs = self.idealPositions[:, 0]
        ys = self.idealPositions[:, 1]
        zs = self.idealPositions[:, 2]

        ax.scatter(xs, ys, zs, marker='D',
                   label="Ideal goal positions.", color='b')
        for i in range(0, self.num_drones):
            ax.text(xs[i], ys[i], zs[i], str(i), ha='left', c='b')

        # Plot adjusted positions.
        xs = self.adjustedPositions[:, 0]
        ys = self.adjustedPositions[:, 1]
        zs = self.adjustedPositions[:, 2]

        ax.scatter(xs, ys, zs, marker='s',
                   label="Adjusted goal positions.", color='orange')
        for i in range(0, self.num_drones):
            ax.text(xs[i], ys[i], zs[i], str(i), ha='left', c='orange')

        # Plot initial grid.
        xs = self.initialGrid[:, 0]
        ys = self.initialGrid[:, 1]
        zs = self.initialGrid[:, 2]

        ax.scatter(xs, ys, zs, marker='o', label="Initial grid.", color='g')
        for i in range(0, self.num_drones):
            ax.text(xs[i], ys[i], zs[i], str(i), ha='left', c='g')

        # Plot flight space.
        center = np.mean(self.dims[:, :2], axis=0)
        ax.plot(center[0], center[1], '^',
                label='Flight space center.', color='red')

        ax.plot([self.dims[0][0], self.dims[0][0], self.dims[1][0], self.dims[1][0], self.dims[0][0], self.dims[0][0], self.dims[0][0], self.dims[1][0], self.dims[1][0], self.dims[0][0]],
                [self.dims[0][1], self.dims[1][1], self.dims[1][1], self.dims[0][1], self.dims[0][1],
                    self.dims[0][1], self.dims[1][1], self.dims[1][1], self.dims[0][1], self.dims[0][1]],
                [self.dims[0][2], self.dims[0][2], self.dims[0][2], self.dims[0][2], self.dims[0][2], self.dims[1][2], self.dims[1][2], self.dims[1][2], self.dims[1][2], self.dims[1][2]], linestyle='dashed', color='black')

        ax.plot([self.dims[1][0], self.dims[1][0]],
                [self.dims[0][1], self.dims[0][1]],
                [self.dims[0][2], self.dims[1][2]], linestyle='dashed', color='black')

        ax.plot([self.dims[0][0], self.dims[0][0]],
                [self.dims[1][1], self.dims[1][1]],
                [self.dims[0][2], self.dims[1][2]], linestyle='dashed', color='black')

        ax.plot([self.dims[1][0], self.dims[1][0]],
                [self.dims[1][1], self.dims[1][1]],
                [self.dims[0][2], self.dims[1][2]], linestyle='dashed', label='Flight space dimensions.', color='black')
        ax.axis('equal')

        # Plot lines between UAVs from initial grid and their assigned waypoints.
        ax.plot([self.initialGrid[0][0], self.droneAssignments[0][0]],
                [self.initialGrid[0][1], self.droneAssignments[0][1]],
                [self.initialGrid[0][2], self.droneAssignments[0][2]], linestyle='dashdot', color="gray", label="Flight path.")
        for i in range(1, self.num_drones):
            ax.plot([self.initialGrid[i][0], self.droneAssignments[i][0]],
                    [self.initialGrid[i][1], self.droneAssignments[i][1]],
                    [self.initialGrid[i][2], self.droneAssignments[i][2]], linestyle='dashdot', color="gray")

        ax.legend()

    def save(self):
        file_name = os.path.basename(self.in_path)
        name = file_name.split('.', 1)[0]

        np.savetxt(self.out_path + name +
                   '_mdf_initial_grid.csv', self.initialGrid, delimiter=",")

        np.savetxt(self.out_path + name +
                   '_mdf_wpts.csv', self.droneAssignments, delimiter=",")
