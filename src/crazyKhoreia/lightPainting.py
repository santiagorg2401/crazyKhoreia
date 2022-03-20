#!/usr/bin/env python3

import itertools
import datetime
import sys
import six
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from cycler import cycler
from crazyKhoreia.crazyKhoreia import crazyKhoreia

sys.modules['sklearn.externals.six'] = six
import mlrose

class lightPainting():
    def __init__(self, MAX_WIDTH, MAX_HEIGHT, MIN_WIDTH, MIN_HEIGHT, in_path, out_path, detail=0.05, speed=1.0, sleepTime=0.1, video=False, led=False):

        self.MAX_WIDTH, self.MAX_HEIGHT, self.MIN_WIDTH, self.MIN_HEIGHT, self.in_path, self.out_path = MAX_WIDTH, MAX_HEIGHT, MIN_WIDTH, MIN_HEIGHT, in_path, out_path
        self.detail, self.speed, self.sleepTime, self.video, self.led = detail, speed, sleepTime, video, led

        self.ck = crazyKhoreia(self.MAX_WIDTH, self.MAX_HEIGHT,
                               self.MIN_WIDTH, self.MIN_HEIGHT, self.in_path, self.led)

        dist_list, ax = self.get_distances(
            self.ck.cnt_scaled, self.ck.width, self.ck.height)

        # best_state = self.find_order(ck.cnt_scaled, dist_list)
        self.wayPoints = self.ck.get_waypoints(
            self.ck.cnt_scaled, self.ck.width, self.ck.height, ax,  best_state=None)
        self.wayPoints = self.ck.clean_waypoints(self.wayPoints, self.detail)
        self.wayPoints = self.ck.trans_waypoints(self.wayPoints)
        self.distance, self.Time = self.calculate_stats()
        self.msg = self.save(self.wayPoints, self.distance,
                             self.Time, self.in_path, self.out_path, self.video)

    def get_distances(self, cnt_scaled, width, height):
        strPoints = np.empty((0, 2))
        endPoints = np.empty((0, 2))
        distances = np.array([])

        # Iterate contours.
        for i in range(0, len(cnt_scaled)):
            x = (cnt_scaled[i][0][0][0] - (width/2))
            y = (cnt_scaled[i][0][0][1] - (height/2))
            strPoints = np.vstack([strPoints, [x, y]])

            x = (cnt_scaled[i][-1][0][0] - (width/2))
            y = (cnt_scaled[i][-1][0][1] - (height/2))
            endPoints = np.vstack([endPoints, [x, y]])

        combs = np.array(
            list(itertools.combinations(range(0, len(cnt_scaled)), 2)))

        for i in combs:
            start_ = i[0]
            end_ = i[1]

            dist = np.linalg.norm(endPoints[start_] - strPoints[end_])
            distances = np.append(distances, dist)

        dist_list = np.array([combs[:, 0], combs[:, 1], distances]).T

        # See https://matplotlib.org/3.5.1/gallery/color/named_colors.html#sphx-glr-gallery-color-named-colors-py for more colors.
        cc = (cycler(color=['purple', 'orange', 'lime', 'royalblue', 'steelblue', 'cyan', 'gold']) +
              cycler(marker=['^', 'o', 's', 'p', '*', 'x', 'D']))

        ax = plt.subplot()
        ax.set_prop_cycle(cc)

        ax.plot(strPoints[:, 0], strPoints[:, 1], 'o', c='b',
                label="Start points.", alpha=0.30, ms=12)
        ax.plot(endPoints[:, 0], endPoints[:, 1], 's', c='r',
                label="End points.", alpha=0.30, ms=12)

        for i in range(0, len(strPoints)):
            ax.arrow(strPoints[:, 0][i], strPoints[:, 1][i], endPoints[:, 0][i] -
                     strPoints[:, 0][i], endPoints[:, 1][i] - strPoints[:, 1][i], ls='--', color='g')
            ax.text(strPoints[:, 0][i], strPoints[:, 1]
                    [i], str(i), ha='left', c='b')
            ax.text(endPoints[:, 0][i], endPoints[:, 1]
                    [i], str(i), ha='right', c='r')

        return dist_list, ax

    def find_order(self, cnt_scaled, dist_list):
        # Initialize fitness function object using dist_list
        fitness_dists = mlrose.TravellingSales(distances=dist_list)

        # Define optimization problem object
        problem_fit = mlrose.TSPOpt(length=len(
            cnt_scaled), fitness_fn=fitness_dists, maximize=False)

        # Solve using genetic algorithm
        best_state, best_fitness, fitness_curve = mlrose.genetic_alg(
            problem_fit, mutation_prob=0.2, max_attempts=100, random_state=2, curve=True)

        #best_state, best_fitness = mlrose.random_hill_climb(problem_fit, max_attempts=100, random_state=2)

        ax = plt.subplot()
        ax.plot(fitness_curve, label='Fitness curve.')
        ax.legend()
        plt.show()

        return best_state

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
        Time = distance/self.speed + len(self.wayPoints)*self.sleepTime

        return distance, Time

    def save(self, wayPoints, distance, Time, in_path, out_path, video):
        file_name = os.path.basename(in_path)
        name = file_name.split('.', 1)[0]

        X = wayPoints[:, 0]
        Y = wayPoints[:, 1]
        Z = wayPoints[:, 2]

        fig, ax = plt.subplots()
        line, = ax.plot(Y, Z, color='#570861')

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
              "\nTake off height: " + str(takeOffHeight) + \
              "\nMinimum coordinates: " + str(minCoords) + \
              "\nMaximum coordinates: " + str(maxCoords) + \
              "\nNumber of waypoints: " + str(len(wayPoints)) + \
              "\nTotal distance: " + str(distance) + " meters." + \
              "\nTotal time: " + str(datetime.timedelta(seconds=Time))

        print(msg)

        # Plot points.
        plt.plot(wayPoints[:, 1], wayPoints[:, 2], 'o')
        plt.show()
        return msg
