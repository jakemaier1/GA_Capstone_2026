#!/usr/bin/env python
# encoding: utf-8
import time
import cv2 as cv
import numpy as np
from typing import NamedTuple
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class LineCentroid(NamedTuple):
  center: int
  width: int

# Fraction of the image height (from the top) that is blacked out before line detection.
# 0.0 = use the whole frame, 0.5 = ignore the top half, 1.0 = ignore everything.
LINE_FOLLOW_TOP_CROP_FRACTION = 0.2
# Fractions of the image height (from the top) at which a 1D row of pixels is sampled
# from the binary image.
# DETECT row: farther from the camera; used to spot upcoming turn options (forks/intersections).
DETECT_LINE_SAMPLE_ROW_FRACTION = 0.4
# ACT row: closer to the camera; used for the line we actually steer on (low latency).
ACT_LINE_SAMPLE_ROW_FRACTION = 0.7
# blobs of this size and larger are considered significant
GAP_THRESH = 4
LINE_THRESH = 4

def write_HSV(wf_path, value):
    with open(wf_path, "w") as wf:
        wf_str = str(value[0][0]) + ', ' + str(
            value[0][1]) + ', ' + str(value[0][2]) + ', ' + str(
            value[1][0]) + ', ' + str(value[1][1]) + ', ' + str(
            value[1][2])
        wf.write(wf_str)
        wf.flush()

def read_HSV(rf_path):
    rf = open(rf_path, "r+")
    line = rf.readline()
    if len(line) == 0: return ()
    list = line.split(',')
    if len(list) != 6: return ()
    hsv = ((int(list[0]), int(list[1]), int(list[2])),
           (int(list[3]), int(list[4]), int(list[5])))
    rf.flush()
    return hsv


# Define the function, the first parameter is the zoom ratio, and the second parameter is a tuple or list of pictures to be displayed
def ManyImgs(scale, imgarray):
    rows = len(imgarray)  # Length of tuple or list
    cols = len(imgarray[0])
    # If imgarray is a list, return the number of channels of the first image in the list, if it is a tuple, return the length of the first list contained in the tuple

    # print("rows=", rows, "cols=", cols)
    # Determine whether the type of imgarray[0] is list
    # It is a list, indicating that imgarray is a tuple and needs to be displayed verticallyIt is a list, indicating that imgarray is a tuple and needs to be displayed vertically
    rowsAvailable = isinstance(imgarray[0], list)
    # The width and height of the first picture
    width = imgarray[0][0].shape[1]
    height = imgarray[0][0].shape[0]
    # print("width=", width, "height=", height)
    # If the incoming is a tuple
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                # Traverse the tuple, if it is the first image, do not transform
                if imgarray[x][y].shape[:2] == imgarray[0][0].shape[:2]:
                    imgarray[x][y] = cv.resize(imgarray[x][y], (0, 0), None, scale, scale)
                # Transform other matrices to the same size as the first image, and the zoom ratio is scale
                else:
                    imgarray[x][y] = cv.resize(imgarray[x][y], (imgarray[0][0].shape[1], imgarray[0][0].shape[0]), None,
                                               scale, scale)
                # If the image is grayscale, convert it to color display
                if len(imgarray[x][y].shape) == 2:
                    imgarray[x][y] = cv.cvtColor(imgarray[x][y], cv.COLOR_GRAY2BGR)
        # Create a blank canvas, the same size as the first picture
        imgBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imgBlank] * rows
        # The same size as the first picture, and the same number of horizontal blank images as the tuple contains the list
        for x in range(0, rows):
            # 将元组里第x个列表水平排列
            # Arrange the x-th list in the tuple horizontally
            hor[x] = np.hstack(imgarray[x])
        ver = np.vstack(hor)
        # Concatenate different lists vertically
    # If the incoming is a list
    else:
        # Transformation operation, same as before
        for x in range(0, rows):
            if imgarray[x].shape[:2] == imgarray[0].shape[:2]:
                imgarray[x] = cv.resize(imgarray[x], (0, 0), None, scale, scale)
            else:
                imgarray[x] = cv.resize(imgarray[x], (imgarray[0].shape[1], imgarray[0].shape[0]), None, scale, scale)
            if len(imgarray[x].shape) == 2:
                imgarray[x] = cv.cvtColor(imgarray[x], cv.COLOR_GRAY2BGR)
        # Arrange the list horizontally
        hor = np.hstack(imgarray)
        ver = hor
    return ver


class color_follow:
    def __init__(self):
        '''
	    Initialize some parameters
        '''
        self.binary = None
        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0
        self.detect_lines = []
        self.act_lines = []

    def detect_diverged_lines(self, px_arr):
        arr = np.array(px_arr)
        arr[arr > 0] = 1
        pad_arr = np.pad(arr, 1)
        diff_arr = np.diff(pad_arr)
        transitionIndices = np.flatnonzero(diff_arr)
        # start and stop indices of LINES, not GAPS
        startIdxs = transitionIndices[0::2]
        stopIdxs = transitionIndices[1::2]

        # REMOVE INSIGNIFICANT GAPS
        # iterate in reverse to avoid indexing issues when deleting from arr
        for i in range(len(startIdxs) - 2, -1, -1):
            gap_len = startIdxs[i+1] - stopIdxs[i]
            # remove insignificant gaps
            if gap_len < GAP_THRESH:
                startIdxs = np.delete(startIdxs, i+1)
                stopIdxs = np.delete(stopIdxs, i)

        # REMOVE INSIGNIFICANT LINES
        for i in range(len(startIdxs) - 1, -1, -1):
            line_len = stopIdxs[i] - startIdxs[i]
            if line_len < LINE_THRESH:
                startIdxs = np.delete(startIdxs, i)
                stopIdxs = np.delete(stopIdxs, i)
        lines = []
        for i in range(len(startIdxs)):
            width = stopIdxs[i] - startIdxs[i]
            center = (width // 2) + startIdxs[i]
            lines.append(LineCentroid(int(center), int(width)))
        return lines

    def line_follow(self, rgb_img, hsv_msg):
        height, width = rgb_img.shape[:2]
        img = rgb_img.copy()
        img[0:int(height * LINE_FOLLOW_TOP_CROP_FRACTION), 0:width] = 0
        # Convert the image to HSV.
        hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        lower = np.array(hsv_msg[0], dtype="uint8")
        upper = np.array(hsv_msg[1], dtype="uint8")
        # Create a mask based on a specific color range
        mask = cv.inRange(hsv_img, lower, upper)
        color_mask = cv.bitwise_and(hsv_img, hsv_img, mask=mask)
        # Convert the image to grayscale
        gray_img = cv.cvtColor(color_mask, cv.COLOR_RGB2GRAY)
        # Get structure elements of different shapes
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # Morphological closed operation
        gray_img = cv.morphologyEx(gray_img, cv.MORPH_CLOSE, kernel)
        # Image binarization operation
        ret, binary = cv.threshold(gray_img, 10, 255, cv.THRESH_BINARY)
        # Sample two 1D horizontal rows of the binary image and detect the lines in each
        detect_y = int(height * DETECT_LINE_SAMPLE_ROW_FRACTION)
        act_y = int(height * ACT_LINE_SAMPLE_ROW_FRACTION)
        self.detect_lines = self.detect_diverged_lines(binary[detect_y])
        self.act_lines = self.detect_diverged_lines(binary[act_y])
        # Get the set of contour points (coordinates)
        find_contours = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if len(find_contours) == 3: contours = find_contours[1]
        else: contours = find_contours[0]
        if len(contours) != 0:
            areas = []
            for c in range(len(contours)): areas.append(cv.contourArea(contours[c]))
            max_id = areas.index(max(areas))
            max_rect = cv.minAreaRect(contours[max_id])
            max_box = cv.boxPoints(max_rect)
            # Convert box vertices to integer pixel coordinates
            max_box = np.int0(max_box)
            # Draw the smallest rectangle
            cv.drawContours(rgb_img, [max_box], 0, (255, 0, 0), 2)
            (color_x, color_y), color_radius = cv.minEnclosingCircle(max_box)
            # Mark the detected color with the original shape coil
            self.Center_x = int(color_x)
            self.Center_y = int(color_y)
            self.Center_r = int(color_radius)
            cv.circle(rgb_img, (self.Center_x, self.Center_y), 5, (255, 0, 255), -1)
        else:
            self.Center_x = 0
            self.Center_y = 0
            self.Center_r = 0
        return rgb_img, binary, (self.Center_x, self.Center_y, self.Center_r), self.detect_lines, self.act_lines

    def Roi_hsv(self, img, Roi):
        '''
        Get the range of HSV in a certain area获取某一区域的HSV的范围
        :param img: 彩色图
        :param Roi:  (x_min, y_min, x_max, y_max)
        Roi=(290,280,350,340)
        :return: The range of images and HSV such as:(0,0,90)(177,40,150) 图像和HSV的范围 例如：(0,0,90)(177,40,150)
        '''
        H = [];S = [];V = []
        # img = cv.resize(img, (640, 480), )
        # 将彩色图转成HSV
        # Convert color image to HSV
        HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        # 画矩形框
        # Draw a rectangular frame
        # cv.rectangle(img, (Roi[0], Roi[1]), (Roi[2], Roi[3]), (0, 255, 0), 2)
        # 依次取出每行每列的H,S,V值放入容器中
        # Take out the H, S, V values of each row and each column in turn and put them into the container
        for i in range(Roi[0], Roi[2]):
            for j in range(Roi[1], Roi[3]):
                H.append(HSV[j, i][0])
                S.append(HSV[j, i][1])
                V.append(HSV[j, i][2])
        # 分别计算出H,S,V的最大最小
        # Calculate the maximum and minimum of H, S, and V respectively
        H_min = min(H); H_max = max(H)
        S_min = min(S); S_max = 253
        V_min = min(V); V_max = 255
        # HSV范围调整
        # HSV range adjustment
        if H_max + 5 > 255: H_max = 255
        else: H_max += 5
        if H_min - 5 < 0: H_min = 0
        else: H_min -= 5
        if S_min - 20 < 0: S_min = 0
        else: S_min -= 20
        if V_min - 20 < 0: V_min = 0
        else: V_min -= 20
        lowerb = 'lowerb : (' + str(H_min) + ' ,' + str(S_min) + ' ,' + str(V_min) + ')'
        upperb = 'upperb : (' + str(H_max) + ' ,' + str(S_max) + ' ,' + str(V_max) + ')'
        txt1 = 'Learning ...'
        txt2 = 'OK !!!'
        if S_min < 5 or V_min < 5:
            cv.putText(img, txt1, (30, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        else:
            cv.putText(img, txt2, (30, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv.putText(img, lowerb, (150, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        cv.putText(img, upperb, (150, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        hsv_range = ((int(H_min), int(S_min), int(V_min)), (int(H_max), int(S_max), int(V_max)))
        return img, hsv_range


class simplePID:
    '''very simple discrete PID controller'''

    def __init__(self, target, P, I, D):
        '''Create a discrete PID controller
        each of the parameters may be a vector if they have the same length
        Args:
        target (double) -- the target value(s)
        P, I, D (double)-- the PID parameter
        '''
        # check if parameter shapes are compatabile.
        if (not (np.size(P) == np.size(I) == np.size(D)) or ((np.size(target) == 1) and np.size(P) != 1) or (
                np.size(target) != 1 and (np.size(P) != np.size(target) and (np.size(P) != 1)))):
            raise TypeError('input parameters shape is not compatable')
        self.Kp = np.array(P)
        self.Ki = np.array(I)
        self.Kd = np.array(D)
        self.last_error = 0
        self.integrator = 0
        self.timeOfLastCall = None
        self.setPoint = np.array(target)
        self.integrator_max = float('inf')

    def update(self, current_value):
        '''Updates the PID controller.
        Args:
            current_value (double): vector/number of same legth as the target given in the constructor
        Returns:
            controll signal (double): vector of same length as the target
        '''
        current_value = np.array(current_value)
        if (np.size(current_value) != np.size(self.setPoint)):
            raise TypeError('current_value and target do not have the same shape')
        if (self.timeOfLastCall is None):
            # the PID was called for the first time. we don't know the deltaT yet
            # no controll signal is applied
            self.timeOfLastCall = time.perf_counter()
            return np.zeros(np.size(current_value))
        error = self.setPoint - current_value
        P = error
        currentTime = time.perf_counter()
        deltaT = (currentTime - self.timeOfLastCall)
        # integral of the error is current error * time since last update
        self.integrator = self.integrator + (error * deltaT)
        I = self.integrator
        # derivative is difference in error / time since last update
        D = (error - self.last_error) / deltaT
        self.last_error = error
        self.timeOfLastCall = currentTime
        # return controll signal
        return self.Kp * P + self.Ki * I + self.Kd * D
