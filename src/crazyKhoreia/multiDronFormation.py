#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
from mpl_toolkits.mplot3d import Axes3D
from itertools import combinations
import os
from crazyKhoreia._3DIoU import _3DIoU
from crazyKhoreia.crazyKhoreia import crazyKhoreia

class multiDronFormation():
  def __init__(self, MAX_WIDTH, MAX_HEIGHT, MIN_WIDTH, MIN_HEIGHT, MIN_DEPTH, MAX_DEPTH, boxShape, in_path, out_path, nmbr_drones):
    self.MAX_WIDTH, self.MAX_HEIGHT, self.MIN_WIDTH, self.MIN_HEIGHT, self.MIN_DEPTH, self.MAX_DEPTH, self.boxShape, self.in_path, self.out_path = MAX_WIDTH, MAX_HEIGHT, MIN_WIDTH, MIN_HEIGHT, MIN_DEPTH, MAX_DEPTH, boxShape, in_path, out_path
    self.nmbr_drones = nmbr_drones

    self.ck = crazyKhoreia(self.MAX_WIDTH, self.MAX_HEIGHT,
                            self.MIN_WIDTH, self.MIN_HEIGHT, self.in_path)
    
    self.wayPoints = self.ck.get_waypoints()
    self.ck.plot_contour_inspection(self.wayPoints)

    cc = self.get_clusters()
    positions = self.get_idealPositions(cc)
    self.getIoUsppd(positions)

    self.save(positions, in_path, out_path)

    plt.show()

  def get_clusters(self):
    fig, ax0 = plt.subplots()
    kmeans = KMeans(n_clusters = self.nmbr_drones, random_state=0)
    y_pred = kmeans.fit_predict(np.array([self.wayPoints[:,1], self.wayPoints[:,2]]).T)
    cc = kmeans.cluster_centers_

    ax0.scatter(self.wayPoints[:,1], self.wayPoints[:,2], c=y_pred)
    ax0.plot(cc[:,0], cc[:,1], 'o', c='violet', label='Cluster centroids.')
    ax0.legend()
    ax0.set_title("KMeans clusters.")

    return cc

  def get_idealPositions(self, cc):
    positions = np.empty(shape=(0, 3))
    
    positions = np.array([(self.MIN_DEPTH)*np.ones(shape=(len(cc),)), cc[:,0], cc[:,1]]).T

    xs = positions[:,0]
    ys = positions[:,1]
    zs = positions[:,2]

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    ax.scatter(xs, ys, zs)
    ax.set_title("Ideal UAV formation inspection.")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    
    return positions

  def getIoUsppd(self, positions):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    xs = positions[:,0]
    ys = positions[:,1]
    zs = positions[:,2]

    ax.scatter(xs, ys, zs, marker='D', label = "Ideal goal positions.")
    for i in range(0, self.nmbr_drones):
      ax.text(xs[i], ys[i], zs[i], str(i), ha='left', c='b')

    while(1):
      print("Minimizing IoU ...")
      combs = combinations(range(0, self.nmbr_drones), 2)
      UAV_IoU = np.zeros(shape=(self.nmbr_drones, 1))

      for i in combs:
        box1 = [self.boxShape, 0, positions[i[0]]/np.array([1, 1, 2])]
        box2 = [self.boxShape, 0, positions[i[1]]/np.array([1, 1, 2])]
        IoU = _3DIoU(box1, box2)

        UAV_IoU[i[0]] += IoU.IOU_3d
        UAV_IoU[i[1]] += IoU.IOU_3d

      maxIoU = max(UAV_IoU)
      maxUAV = np.where(UAV_IoU == maxIoU)[0][0]
      print("Maximum IoU: " + str(maxIoU))

      if maxIoU != 0:
        if max(positions[:,0]) <= self.MAX_DEPTH:
          positions[maxUAV][0] += self.boxShape[0]
        else:
          print("Formation for " + str(self.nmbr_drones) + " UAVs failed at UAV " + str(maxUAV) + " depth limitations exceeded, please lower the number of UAVs.")
          break
      elif maxIoU ==0:
        print("Interception free formation found.")
        break    

    xs = positions[:,0]
    ys = positions[:,1]
    zs = positions[:,2]

    ax.scatter(xs, ys, zs, marker='s', label = "Adjusted goal positions.")
    for i in range(0, self.nmbr_drones):
      ax.text(xs[i], ys[i], zs[i], str(i), ha='left', c='orange')
    ax.legend()

  def descend(positions):
    pass

  def save(self, positions, in_path, out_path):
    file_name = os.path.basename(in_path)
    name = file_name.split('.', 1)[0]
    
    np.savetxt(out_path + name +
                '_mdf_pos.csv', positions, delimiter=",")
    