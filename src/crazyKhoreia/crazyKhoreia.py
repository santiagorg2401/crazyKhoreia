#!/usr/bin/env python3

import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
from image2gcode.connectedRoboSketch_lib import *


class crazyKhoreia():
    def __init__(self, MAX_WIDTH, MAX_HEIGHT, MIN_WIDTH, MIN_HEIGHT, in_path, led=False):
        self.MAX_WIDTH, self.MAX_HEIGHT, self.MIN_WIDTH, self.MIN_HEIGHT, self.in_path, self.led = MAX_WIDTH, MAX_HEIGHT, MIN_WIDTH, MIN_HEIGHT, in_path, led
        self.cnt_scaled, self.width, self.height = self.process_image()

    def process_image(self):
        img = cv.flip(cv.imread(self.in_path, 0), 0)
        height, width = img.shape[:2]

        fig, (ax0, ax1) = plt.subplots(nrows=1, ncols=2)
        ax0.imshow(img, cmap="gray")
        ax0.set_axis_off()
        ax0.set_title("Original image.")
        ax1.set_axis_off()
        ax1.set_title("Edges image.")

        scale_fact_x, scale_fact_y = getImageScaleFact(img, self.MAX_WIDTH, self.MAX_HEIGHT)  # scale factor for input image

        edges = edgeDetect(img)  # peform canny edge detection on the image

        # vectorization of edges and cleanup of vectors
        contours = vectorizeEdges(edges)

        # scales vectors by whichever scaling factor is larger
        cnt_scaled = scaleVectors(contours, scale_fact_x, scale_fact_y)

        # scales width and height of original image
        width, height = scaleDimensions(
            width, height, scale_fact_x, scale_fact_y)

        ax1.imshow(edges, cmap="gray")
        plt.show()

        return cnt_scaled, width, height

    def get_waypoints(self, cnt_scaled, width, height, ax, best_state=None):
        # Convert vectors (contours) to cartesian points in XYZ [meters] format.
        if self.led == True:
            wayPoints = np.empty((0, 4))
        else:
            wayPoints = np.empty((0, 3))

        if best_state == None:
            best_state = range(0, len(cnt_scaled))

        for i in best_state:                             # Iterate contours.
            x = np.array(cnt_scaled[i])
            x = np.reshape(
                x, (len(cnt_scaled[i]), 2)) - np.array([width/2, height/2])

            if self.led == True:
                led = np.ones(shape=(len(x),))
                led[0] = 0
                stck = np.array(
                    [1.5*np.ones(shape=(len(x),)), x[:, 0], x[:, 1], led]).T
            else:
                stck = np.array(
                    [1.5*np.ones(shape=(len(x),)), x[:, 0], x[:, 1]]).T

            wayPoints = np.vstack([wayPoints, stck])
            ax.plot(x[:, 0], x[:, 1], label="Contour " + str(i), ls='--')

        ax.plot(wayPoints[:, 1], wayPoints[:, 2], c='silver',
                label='wayPoints', alpha=0.8, ls=':', marker=',')
        ax.legend()
        ax.set_title("Contour inspection.")
        plt.show()

        return wayPoints

    def clean_waypoints(self, wayPoints, detail):
        # Clean wayPoints by a detail parameter.
        initialPoints = len(wayPoints)

        for i in range(1, initialPoints):
            if ((i + 1 == len(wayPoints)) | (i == len(wayPoints))):
                break
            elif (np.linalg.norm(wayPoints[:, 0:3][i - 1] - wayPoints[:, 0:3][i]) < detail):
                wayPoints = np.delete(wayPoints, i - 1, 0)

        return wayPoints

    def trans_waypoints(self, wayPoints):
        # Translate the dataset so the minimum distance is at least yMin by zMin meters in YZ plane.

        if self.led == True:
            wayPoints = wayPoints + \
                abs(np.array(
                    [0, min(wayPoints[:, 1]), min(wayPoints[:, 2]), 0]))
            wayPoints = wayPoints + \
                np.array([0, self.MIN_WIDTH, self.MIN_HEIGHT, 0])
        else:
            wayPoints = wayPoints + \
                abs(np.array([0, min(wayPoints[:, 1]), min(wayPoints[:, 2])]))
            wayPoints = wayPoints + \
                np.array([0, self.MIN_WIDTH, self.MIN_HEIGHT])

        return wayPoints
