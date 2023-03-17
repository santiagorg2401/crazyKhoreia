#!/usr/bin/env python3

import datetime
import os

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

from crazyKhoreia.crazyKhoreia import crazyKhoreia


class lightPainting(crazyKhoreia):
    """
    CrazyKhoreia's child class LightPainting performs waypoint processing from CrazyKhoreia's image-based waypoint generation.
    Attributes:
        dims        (array):    2x3 float array containing flight space constraints in the x, y, and z axis [[MIN_X, MIN_Y, MIN_Z],[MAX_X, MAX_Y, MAX_Z]]
        in_path     (str):      Global image's path to process.
        out_path    (str):      File output path.
        detail      (float):    Waypoint resolution.
        speed       (float):    UAV speed.
        sleepTime   (float):    Percentage to estimate flight duration if the UAV stops at each waypoint. TODO: Is this really necessary?
        video       (bool):     Set to export a video animation of the UAV.
        led         (bool):     Whether or not to control LED light relative to out of contour travel. TODO: Is this really necessary?
        
        cnt_scaled  (list):     List of processed contours.
        wpts        (list):     List of k x 3 waypoints matrix plus additional columns.
        distance    (float):    Total flight distance.
        Time        (float):    Total flight time.

    Methods:
        clean_waypoints():
            Removes waypoints that are at a certain distance from each other according to a detail parameter.
        update(numb, x, y, line):
            Helper function to animate the video.
        calculate_stats():
            Calculate flight metrics such as total distance and time.
        save():
            If set computes animation, saves files to set location and prints summary.
    """

    def __init__(self, dims, in_path, out_path, detail=0.05, speed=1.0, sleepTime=1.5, video=False, led=False):
        super().__init__(dims, in_path, led)

        self.dims, self.in_path, self.out_path = dims, in_path, out_path
        self.detail, self.speed, self.sleepTime, self.video, self.led = detail, speed, sleepTime, video, led

        self.wpts = self.get_waypoints()
        self.clean_waypoints()

        self.distance, self.Time = self.calculate_stats()
        self.save()

        self.plot_contour_inspection(self.wpts)

        plt.show()

    def clean_waypoints(self):
        # Clean wayPoints by a detail parameter.
        initialPoints = len(self.wpts)

        for j in range(0, 10):
            for i in range(1, initialPoints):
                if ((i + 1 == len(self.wpts)) | (i == len(self.wpts))):
                    break
                elif self.led == True:
                    if (np.linalg.norm(self.wpts[:, 0:3][i - 1] - self.wpts[:, 0:3][i]) < self.detail) & (self.wpts[:, 3][i] == 1):
                        self.wpts = np.delete(self.wpts, i, 0)
                elif self.led == False:
                    if (np.linalg.norm(self.wpts[:, 0:3][i - 1] - self.wpts[:, 0:3][i]) < self.detail):
                        self.wpts = np.delete(self.wpts, i, 0)

    def update(self, num, x, y, line):
        line.set_data(x[:num], y[:num])
        return line,

    def calculate_stats(self):
        distance = 0
        takeOffHeight = self.wpts[0][2]
        initialPos = np.array(
            self.wpts[:, 0:3][0] - np.array([0, 0, takeOffHeight]))

        for i in range(0, len(self.wpts)):
            if i == 0:
                distance += abs(np.linalg.norm(initialPos -
                                self.wpts[:, 0:3][0]))
            else:
                distance += abs(np.linalg.norm(
                    self.wpts[:, 0:3][i - 1] - self.wpts[:, 0:3][i]))

        Time = distance/self.speed*self.sleepTime

        return distance, Time

    def save(self):
        file_name = os.path.basename(self.in_path)
        name = file_name.split('.', 1)[0]

        X = self.wpts[:, 0]
        Y = self.wpts[:, 1]
        Z = self.wpts[:, 2]

        # Plot points.
        fig2, ax1 = plt.subplots()
        ax1.plot(self.wpts[:, 1], self.wpts[:, 2], 'o',
                 c='blueviolet', label="waypoints.")
        ax1.plot(Y, Z, color='#570861')
        ax1.set_title("UAV path.")

        fig, ax = plt.subplots()
        line, = ax.plot(Y, Z, color='#570861')
        ax.set_title("UAV path animation.")

        if self.video == True:
            ani = animation.FuncAnimation(fig, self.update, len(Y), fargs=[Y, Z, line],
                                          interval=50, blit=True)
            ani.save(self.out_path + name + '_lp_video.mp4')
            plt.show()

        np.savetxt(self.out_path + name +
                   '_lp_wpts.csv', self.wpts, delimiter=",")

        minCoords = np.array([min(X), min(Y), min(Z)])
        maxCoords = np.array([max(X), max(Y), max(Z)])
        takeOffHeight = self.wpts[0][2]

        initialPos = self.wpts[:, 0:3][0] - \
            np.array([0, 0, takeOffHeight])

        msg = "Choreography ready!, please read the following information:" + \
              "\nInitial position: " + str(initialPos) + \
              "\nTake off heigth: " + str(takeOffHeight) + \
              "\nMinimum coordinates: " + str(minCoords) + \
              "\nMaximum coordinates: " + str(maxCoords) + \
              "\nNumber of waypoints: " + str(len(self.wpts)) + \
              "\nTotal distance: " + str(self.distance) + " meters." + \
              "\nTotal time: " + str(datetime.timedelta(seconds=self.Time))

        print(msg)
