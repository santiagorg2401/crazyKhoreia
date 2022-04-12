#!/usr/bin/env python3

import datetime
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from crazyKhoreia.crazyKhoreia import crazyKhoreia


class lightPainting():
    def __init__(self, MAX_WIDTH, MAX_HEIGHT, MIN_WIDTH, MIN_HEIGHT, in_path, out_path, detail=0.05, speed=1.0, sleepTime=1.5, video=False, led=False):

        self.MAX_WIDTH, self.MAX_HEIGHT, self.MIN_WIDTH, self.MIN_HEIGHT, self.in_path, self.out_path = MAX_WIDTH, MAX_HEIGHT, MIN_WIDTH, MIN_HEIGHT, in_path, out_path
        self.detail, self.speed, self.sleepTime, self.video, self.led = detail, speed, sleepTime, video, led

        self.ck = crazyKhoreia(self.MAX_WIDTH, self.MAX_HEIGHT,
                               self.MIN_WIDTH, self.MIN_HEIGHT, self.in_path, self.led)

        self.wayPoints = self.ck.get_waypoints()
        self.wayPoints = self.clean_waypoints(self.wayPoints, self.detail)

        self.distance, self.Time = self.calculate_stats()
        self.msg = self.save(self.wayPoints, self.distance,
                             self.Time, self.in_path, self.out_path, self.video)

        self.ck.plot_contour_inspection(self.wayPoints)

        plt.show()

    def clean_waypoints(self, wayPoints, detail):
        # Clean wayPoints by a detail parameter.
        initialPoints = len(wayPoints)

        for j in range(0, 10):
            for i in range(1, initialPoints):
                if ((i + 1 == len(wayPoints)) | (i == len(wayPoints))):
                    break
                elif (np.linalg.norm(wayPoints[:, 0:3][i - 1] - wayPoints[:, 0:3][i]) < detail) & (wayPoints[:, 3][i] == 1):
                    wayPoints = np.delete(wayPoints, i, 0)

        return wayPoints

    def update(self, num, x, y, line):
        line.set_data(x[:num], y[:num])
        return line,

    def calculate_stats(self):
        distance = 0
        takeOffHeight = self.wayPoints[0][2]
        initialPos = np.array(
            self.wayPoints[:, 0:3][0] - np.array([0, 0, takeOffHeight]))

        for i in range(0, len(self.wayPoints)):
            if i == 0:
                distance += abs(np.linalg.norm(initialPos -
                                self.wayPoints[:, 0:3][0]))
            else:
                distance += abs(np.linalg.norm(
                    self.wayPoints[:, 0:3][i - 1] - self.wayPoints[:, 0:3][i]))

        Time = distance/self.speed*self.sleepTime

        return distance, Time

    def save(self, wayPoints, distance, Time, in_path, out_path, video):
        file_name = os.path.basename(in_path)
        name = file_name.split('.', 1)[0]

        X = wayPoints[:, 0]
        Y = wayPoints[:, 1]
        Z = wayPoints[:, 2]

        # Plot points.
        fig2, ax1 = plt.subplots()
        ax1.plot(wayPoints[:, 1], wayPoints[:, 2], 'o',
                 c='blueviolet', label="waypoints.")
        ax1.plot(Y, Z, color='#570861')
        ax1.set_title("UAV path.")

        fig, ax = plt.subplots()
        line, = ax.plot(Y, Z, color='#570861')
        ax.set_title("UAV path animation.")

        if video == True:
            ani = animation.FuncAnimation(fig, self.update, len(Y), fargs=[Y, Z, line],
                                          interval=50, blit=True)
            ani.save(out_path + name + '_lp_video.mp4')
            plt.show()

        np.savetxt(out_path + name +
                   '_lp_wpts.csv', wayPoints, delimiter=",")

        minCoords = np.array([min(X), min(Y), min(Z)])
        maxCoords = np.array([max(X), max(Y), max(Z)])
        takeOffHeight = wayPoints[0][2]

        initialPos = wayPoints[:, 0:3][0] - np.array([0, 0, takeOffHeight])

        msg = "Choreography ready!, please read the following information:" + \
              "\nInitial position: " + str(initialPos) + \
              "\nTake off heigth: " + str(takeOffHeight) + \
              "\nMinimum coordinates: " + str(minCoords) + \
              "\nMaximum coordinates: " + str(maxCoords) + \
              "\nNumber of waypoints: " + str(len(wayPoints)) + \
              "\nTotal distance: " + str(distance) + " meters." + \
              "\nTotal time: " + str(datetime.timedelta(seconds=Time))

        print(msg)

        return msg
