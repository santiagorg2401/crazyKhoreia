#!/usr/bin/env python3

import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
from cycler import cycler


class crazyKhoreia():
    def __init__(self, MAX_WIDTH, MAX_HEIGHT, MIN_WIDTH, MIN_HEIGHT, in_path, led=False):
        self.MAX_WIDTH, self.MAX_HEIGHT, self.MIN_WIDTH, self.MIN_HEIGHT, self.in_path, self.led = MAX_WIDTH, MAX_HEIGHT, MIN_WIDTH, MIN_HEIGHT, in_path, led

        # Read image.
        self.img = cv.imread(self.in_path)

        # Proccess image and get contours from it.
        contours = self.process_image()

        # Proccess the contours and get its parameters relative to the input image.
        self.cnt_scaled, self.ImgShape_X, self.ImgShape_Y = self.process_contours(
            contours)

    def process_image(self):
        # Flip image.
        img_flipped = cv.flip(self.img, 0)

        # Invert the input image before detecting contours (see: https://stackoverflow.com/questions/29329866/how-to-avoid-detecting-image-frame-when-using-findcontours).
        img_flipped = 255 - img_flipped

        # Convert the image from BGR to grayscale.
        im_gray = cv.cvtColor(img_flipped, cv.COLOR_BGR2GRAY)

        # Binarize the grayscale image.
        th, img_bw = cv.threshold(im_gray, 128, 192, cv.THRESH_OTSU)

        # Find countours.
        contours, hierarchy = cv.findContours(
            image=img_bw, mode=cv.RETR_CCOMP, method=cv.CHAIN_APPROX_SIMPLE)

        # Create subplots for original image and image with contours visualization.
        fig, (ax0, ax1) = plt.subplots(nrows=1, ncols=2)

        # img[..., ::-1] reverts the image's channel order from BGR to RGB so it can be correctly displayed by plt.imshow()
        ax0.imshow(self.img[..., ::-1])
        ax0.set_axis_off()
        ax0.set_title("Original image.")
        ax1.set_axis_off()
        ax1.set_title("Image with contours.")

        # Draw contours on original image.
        img_with_contour = cv.drawContours(
            image=255 - img_flipped, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=3, lineType=cv.LINE_AA)

        ax1.imshow(cv.flip(img_with_contour[..., ::-1], 0))

        return contours

    def process_contours(self, contours):
        # Get image's dimensions.
        ImgShape_Y, ImgShape_X = self.img.shape[:2]

        # Get image's scale factor from its dimensions and user's parameters.
        scale_fact_x = ImgShape_X/(self.MAX_WIDTH - self.MIN_WIDTH)
        scale_fact_y = ImgShape_Y/(self.MAX_HEIGHT - self.MIN_HEIGHT)

        # Scale contours if needed to satisfy maximum dimensions requirements.
        cnt_scaled = []

        for cnt in contours:
            cnt = cnt - [ImgShape_X/2, ImgShape_Y/2]
            if(scale_fact_x > 1.0 or scale_fact_y > 1.0):
                if(scale_fact_x > scale_fact_y):
                    cnt_scaled.append(cnt*(1.0/scale_fact_x))
                else:
                    cnt_scaled.append(cnt*(1.0/scale_fact_y))

        # Adjust image parameters relative to its scaled contours.
        if(scale_fact_x > scale_fact_y):
            ImgShape_X /= scale_fact_x
            ImgShape_Y /= scale_fact_x
        if(scale_fact_y > scale_fact_x):
            ImgShape_X /= scale_fact_y
            ImgShape_Y /= scale_fact_y

       # Translate the scaled contours to satisfy minimum dimensions requirements.
        minX = []
        minY = []

        for cnt in cnt_scaled:
            x = np.array(cnt)
            x = np.reshape(x, (len(cnt), 2))
            minX.append(min(x[:, 0]))
            minY.append(min(x[:, 1]))

        minX = abs(min(minX))
        minY = abs(min(minY))

        cnt_scaled_x = []
        for cnt in cnt_scaled:
            cnt_x = cnt + [minX, minY] + [self.MIN_WIDTH, self.MIN_HEIGHT]
            cnt_scaled_x.append(cnt_x)

        cnt_scaled = cnt_scaled_x

        return cnt_scaled, ImgShape_X, ImgShape_Y

    def plot_contour_inspection(self, wayPoints):
        strPoints = np.empty((0, 2))
        endPoints = np.empty((0, 2))

        # See https://matplotlib.org/3.5.1/gallery/color/named_colors.html#sphx-glr-gallery-color-named-colors-py for more colors.
        cc = (cycler(color=['purple', 'orange', 'lime', 'royalblue', 'steelblue', 'cyan', 'gold']) +
              cycler(marker=['^', 'o', 's', 'p', '*', 'x', 'D']))

        fig, ax = plt.subplots()
        ax.set_prop_cycle(cc)

        # Iterate contours.
        for i in range(0, len(self.cnt_scaled)):
            x = np.array(self.cnt_scaled[i])
            x = np.reshape(x, (len(self.cnt_scaled[i]), 2))

            strPoints = np.vstack([strPoints, [x[0, 0], x[0, 1]]])
            endPoints = np.vstack([endPoints, [x[-1, 0], x[-1, 1]]])

            ax.plot(x[:, 0], x[:, 1], label="Contour " + str(i), ls='--')

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

        ax.plot(wayPoints[:, 1], wayPoints[:, 2], c='silver',
                label='wayPoints', alpha=0.8, ls=':', marker=',')
        ax.legend()
        ax.set_title("Contour inspection.")

    def get_waypoints(self, best_state=None):
        # Convert vectors (contours) to cartesian points in XYZ [meters] format.
        if self.led == True:
            wayPoints = np.empty((0, 4))
        else:
            wayPoints = np.empty((0, 3))

        best_state = range(0, len(self.cnt_scaled))

        for i in best_state:                             # Iterate contours.
            x = np.array(self.cnt_scaled[i])
            x = np.reshape(x, (len(self.cnt_scaled[i]), 2))

            if self.led == True:
                led = np.ones(shape=(len(x),))
                led[0] = 0
                stck = np.array(
                    [1.5*np.ones(shape=(len(x),)), x[:, 0], x[:, 1], led]).T
            else:
                stck = np.array(
                    [1.5*np.ones(shape=(len(x),)), x[:, 0], x[:, 1]]).T

            wayPoints = np.vstack([wayPoints, stck])

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
