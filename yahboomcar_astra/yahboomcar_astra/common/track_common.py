#!/usr/bin/env python
# encoding: utf-8
import time
import cv2 as cv
import numpy as np
import threading

class Tracker(object):
    '''
    追踪者模块,用于追踪指定目标
    Tracker module, used to track the specified target
    '''
    def __init__(self, tracker_type="BOOSTING"):
        '''
        初始化追踪器种类
        Initialize the type of tracker
        '''
        # 获得opencv版本
	# Get the opencv version
        (major_ver, minor_ver, subminor_ver) = (cv.__version__).split('.')
        self.tracker_type = tracker_type
        self.isWorking = False
        # 构造追踪器
	# Construct tracker
        if int(major_ver) < 3: self.tracker = cv.Tracker_create(tracker_type)
        else:
            if tracker_type == 'BOOSTING': self.tracker = cv.TrackerBoosting_create()
            if tracker_type == 'MIL': self.tracker = cv.TrackerMIL_create()
            if tracker_type == 'KCF': self.tracker = cv.TrackerKCF_create()
            if tracker_type == 'TLD': self.tracker = cv.TrackerTLD_create()
            if tracker_type == 'MEDIANFLOW': self.tracker = cv.TrackerMedianFlow_create()
            if tracker_type == 'GOTURN': self.tracker = cv.TrackerGOTURN_create()
            if tracker_type == 'MOSSE': self.tracker = cv.TrackerMOSSE_create()
            if tracker_type == "CSRT": self.tracker = cv.TrackerCSRT_create()

    def initWorking(self, frame, box):
        '''
        Tracker work initialization 追踪器工作初始化
        frame:初始化追踪画面
        box:追踪的区域
        '''
        if not self.tracker: raise Exception("Tracker is not initialized")
        status = self.tracker.init(frame, box)
        self.coord = box
        self.isWorking = True

    def track(self, frame):
        if self.isWorking:
            status, self.coord = self.tracker.update(frame)
            if status:
                p1 = (int(self.coord[0]), int(self.coord[1]))
                p2 = (int(self.coord[0] + self.coord[2]), int(self.coord[1] + self.coord[3]))
                cv.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
                return frame, p1, p2
            else:
                # 跟踪失败
		# Tracking failed
                cv.putText(frame, "Tracking failure detected", (100, 80), cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
                return frame, (0, 0), (0, 0)
        else: return frame, (0, 0), (0, 0)
