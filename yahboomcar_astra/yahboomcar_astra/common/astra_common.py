#!/usr/bin/env python
# encoding: utf-8
import time
import cv2 as cv
import numpy as np
import threading


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

def draw_tags(image, tags, corners_color=(0, 125, 255), center_color=(0, 255, 0)):
    for tag in tags:
        corners = tag.corners.astype(int)
        center = tag.center.astype(int)
        cv.putText(image, "%d"%tag.tag_id, (int(center[0] - (7 * len("%d"%tag.tag_id))), int(center[1]-10)), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        if corners_color is not None:
            for p in corners:
                cv.circle(image, tuple(p.tolist()), 5, corners_color, -1)
        if center_color is not None:
            cv.circle(image, tuple(center.tolist()), 8, center_color, -1)
    return image

# 定义函数，第一个参数是缩放比例，第二个参数是需要显示的图片组成的元组或者列表
# Define the function, the first parameter is the zoom ratio, and the second parameter is a tuple or list of pictures to be displayed
def ManyImgs(scale, imgarray):
    rows = len(imgarray)  # 元组或者列表的长度 Length of tuple or list
    cols = len(imgarray[0])  # 如果imgarray是列表，返回列表里第一幅图像的通道数，如果是元组，返回元组里包含的第一个列表的长度
    # If imgarray is a list, return the number of channels of the first image in the list, if it is a tuple, return the length of the first list contained in the tuple
    # print("rows=", rows, "cols=", cols)
    # 判断imgarray[0]的类型是否是list,
    # 是list，表明imgarray是一个元组，需要垂直显示
    # Determine whether the type of imgarray[0] is list,
    # It is a list, indicating that imgarray is a tuple and needs to be displayed vertically
    rowsAvailable = isinstance(imgarray[0], list)
    # 第一张图片的宽高
    # The width and height of the first picture
    width = imgarray[0][0].shape[1]
    height = imgarray[0][0].shape[0]
    # print("width=", width, "height=", height)
    # 如果传入的是一个元组
    # If the incoming is a tuple
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                # 遍历元组，如果是第一幅图像，不做变换
                # Traverse the tuple, if it is the first image, do not transform
                if imgarray[x][y].shape[:2] == imgarray[0][0].shape[:2]:
                    imgarray[x][y] = cv.resize(imgarray[x][y], (0, 0), None, scale, scale)
                # 将其他矩阵变换为与第一幅图像相同大小，缩放比例为scale
                # Transform other matrices to the same size as the first image, and the zoom ratio is scale
                else:
                    imgarray[x][y] = cv.resize(imgarray[x][y], (imgarray[0][0].shape[1], imgarray[0][0].shape[0]), None,
                                               scale, scale)
                # 如果图像是灰度图，将其转换成彩色显示
                # If the image is grayscale, convert it to color display
                if len(imgarray[x][y].shape) == 2:
                    imgarray[x][y] = cv.cvtColor(imgarray[x][y], cv.COLOR_GRAY2BGR)
        # 创建一个空白画布，与第一张图片大小相同
        # Create a blank canvas, the same size as the first picture
        imgBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imgBlank] * rows  # 与第一张图片大小相同，与元组包含列表数相同的水平空白图像
        # The same size as the first picture, and the same number of horizontal blank images as the tuple contains the list
        for x in range(0, rows):
            # 将元组里第x个列表水平排列
            # Arrange the x-th list in the tuple horizontally
            hor[x] = np.hstack(imgarray[x])
        ver = np.vstack(hor)  # 将不同列表垂直拼接 Concatenate different lists vertically
    # 如果传入的是一个列表 If the incoming is a list
    else:
        # 变换操作，与前面相同
        # Transformation operation, same as before
        for x in range(0, rows):
            if imgarray[x].shape[:2] == imgarray[0].shape[:2]:
                imgarray[x] = cv.resize(imgarray[x], (0, 0), None, scale, scale)
            else:
                imgarray[x] = cv.resize(imgarray[x], (imgarray[0].shape[1], imgarray[0].shape[0]), None, scale, scale)
            if len(imgarray[x].shape) == 2:
                imgarray[x] = cv.cvtColor(imgarray[x], cv.COLOR_GRAY2BGR)
        # 将列表水平排列
        # Arrange the list horizontally
        hor = np.hstack(imgarray)
        ver = hor
    return ver

def is_regular_square(points, angle_threshold=30, aspect_threshold=0.8):
    """
    检查四边形是否为规则方形 / Check if a quadrilateral is a regular square
    :param points: 四个顶点坐标 (4x2) / Four vertex coordinates (4x2)
    :param angle_threshold: 角度偏差阈值(度) / Angle deviation threshold (degrees)
    :param aspect_threshold: 宽高比差异阈值 / Aspect ratio difference threshold
    :return: 是否为规则方形 / Whether it is a regular square
    """
    # 计算四边形的四条边 / Calculate the four sides of the quadrilateral
    edges = []
    for i in range(4):
        p1 = points[i]
        p2 = points[(i + 1) % 4]
        edges.append(np.linalg.norm(p2 - p1))
    
    # 安全计算对边比例 - 避免除以零错误 / Safely calculate opposite side ratios - avoid division by zero
    ratios = []
    for i in range(2):  # 计算两对对边 / Calculate two pairs of opposite sides
        a = edges[i]
        b = edges[i+2]
        
        # 确保两边都有有效长度 / Ensure both sides have valid lengths
        if a < 1e-5 or b < 1e-5:
            return False
        
        # 计算对边比例（取较小值保证在0-1之间） / Calculate opposite side ratio (take min value to ensure 0-1)
        ratio = min(a/b, b/a)
        ratios.append(ratio)
    
    # 检查对边比例是否在阈值范围内 / Check if opposite side ratios are within threshold
    if ratios[0] < aspect_threshold or ratios[1] < aspect_threshold:
        return False
    
    # 计算四个内角 / Calculate the four internal angles
    angles = []
    for i in range(4):
        prev_idx = (i - 1) % 4
        next_idx = (i + 1) % 4
        
        # 创建两个向量 / Create two vectors
        vec1 = points[prev_idx] - points[i]
        vec2 = points[next_idx] - points[i]
        
        # 计算向量夹角 / Calculate the angle between vectors
        dot_product = np.dot(vec1, vec2)
        norm_product = np.linalg.norm(vec1) * np.linalg.norm(vec2)
        
        # 避免除以零 / Avoid division by zero
        if norm_product < 1e-5:
            return False
        
        cos_angle = dot_product / norm_product
        # 防止浮点误差导致超出[-1,1]范围 / Prevent floating point error causing out of [-1,1] range
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle_rad = np.arccos(cos_angle)
        angle_deg = np.degrees(angle_rad)
        angles.append(angle_deg)
    
    # 检查角度是否接近90度 / Check if angles are close to 90 degrees
    angle_deviation = max(abs(angle - 90) for angle in angles)
    return angle_deviation < angle_threshold

def detect_qrcode(image):
    """
    识别图像中的二维码并返回边界框 / Detect QR code in image and return bounding box
    :param image: 输入图像 (BGR格式) / Input image (BGR format)
    :return: 
        image: 处理后的图像（带识别结果的可视化）或 None（未检测到时） / Processed image (with visualization) or None (not detected)
        bbox: 二维码边界框 (x, y, w, h) 或 (0, 0, 0, 0)（未检测到时） / QR code bounding box (x, y, w, h) or (0,0,0,0) (not detected)
    """
    # 初始化二维码检测器 / Initialize QR code detector
    qr_decoder = cv.QRCodeDetector()
    
    # 检测并解码二维码 / Detect and decode QR code
    decoded_text, points, _ = qr_decoder.detectAndDecode(image)
    
    if points is not None:
        points = points.astype(int).reshape(-1, 2)
        
        contour_area = cv.contourArea(points)
        if contour_area <= 0:
            return None, (0, 0, 0, 0)

        # 检查是否为规则方形 / Check if it's a regular square
        if not is_regular_square(points.astype(float)):
            return None, (0, 0, 0, 0)
        
        # 计算二维码中心点坐标 / Calculate QR code center coordinates
        center_x = int(np.mean(points[:, 0]))
        center_y = int(np.mean(points[:, 1]))
        
        # 计算边界框 (x, y, w, h) / Calculate bounding box (x, y, w, h)
        x, y, w, h = cv.boundingRect(points)
        
        # 根据解码结果选择颜色 / Choose color based on decoding result
        if decoded_text:
            color = (0, 255, 0)  # 绿色 - 解码成功 / Green - decoding successful
            text = decoded_text
        else:
            color = (0, 0, 255)  # 红色 - 检测到但解码失败 / Red - detected but decoding failed
            text = "Decoding Failed"
        
        # 绘制二维码边界框 / Draw QR code bounding box
        for i in range(len(points)):
            cv.line(image, tuple(points[i]), tuple(points[(i+1) % len(points)]), color, 3)
        
        # 在中心点绘制蓝色标记 / Draw blue marker at center point
        cv.circle(image, (center_x, center_y), 8, (255, 0, 0), -1)
        
        # 在二维码上方显示解码内容 / Display decoded content above QR code
        text_position = (points[0][0], points[0][1] - 15)
        cv.putText(image, text, text_position, 
                    cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        return image, (x, y, w, h)
    
    return None, (0, 0, 0, 0)
    

class color_follow:
    def __init__(self):
        '''
        初始化一些参数
	    Initialize some parameters
        '''
        self.Center_x = 0
        self.Center_y = 0
        self.Center_r = 0

    def object_follow(self, img, hsv_msg):
        src = img.copy()
        # 由颜色范围创建NumPy数组
        # Create NumPy array from color range
        src = cv.cvtColor(src, cv.COLOR_BGR2HSV)
        lower = np.array(hsv_msg[0], dtype="uint8")
        upper = np.array(hsv_msg[1], dtype="uint8")
        # 根据特定颜色范围创建mask
        # Create a mask based on a specific color range
        mask = cv.inRange(src, lower, upper)
        color_mask = cv.bitwise_and(src, src, mask=mask)
        # 将图像转为灰度图
        # Convert the image to grayscale
        gray_img = cv.cvtColor(color_mask, cv.COLOR_RGB2GRAY)
        # 获取不同形状的结构元素
        # Get structure elements of different shapes
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # 形态学闭操作
        # Morphological closed operation
        gray_img = cv.morphologyEx(gray_img, cv.MORPH_CLOSE, kernel)
        # 图像二值化操作
        # Image binarization operation
        ret, binary = cv.threshold(gray_img, 10, 255, cv.THRESH_BINARY)
        # 获取轮廓点集(坐标)
        # Get the set of contour points (coordinates)
        find_contours = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if len(find_contours) == 3:
            contours = find_contours[1]
        else:
            contours = find_contours[0]
        if len(contours) != 0:
            areas = []
            for c in range(len(contours)): areas.append(cv.contourArea(contours[c]))
            max_id = areas.index(max(areas))
            max_rect = cv.minAreaRect(contours[max_id])
            max_box = cv.boxPoints(max_rect)
            max_box = np.int0(max_box)
            (color_x, color_y), color_radius = cv.minEnclosingCircle(max_box)
            # 将检测到的颜色用原形线圈标记出来
            # Mark the detected color with the original shape coil
            self.Center_x = int(color_x)
            self.Center_y = int(color_y)
            self.Center_r = int(color_radius)
            cv.circle(img, (self.Center_x, self.Center_y), self.Center_r, (255, 0, 255), 2)
            cv.circle(img, (self.Center_x, self.Center_y), 2, (0, 0, 255), -1)
        else:
            self.Center_x = 0
            self.Center_y = 0
            self.Center_r = 0
        return img, binary, (self.Center_x, self.Center_y, self.Center_r)

    def Roi_hsv(self, img, Roi):
        '''
        获取某一区域的HSV的范围
        Get the range of HSV in a certain area
        :param img: Color map 彩色图 
        :param Roi:  (x_min, y_min, x_max, y_max)
        Roi=(290,280,350,340)
        :return: 图像和HSV的范围 例如：(0,0,90)(177,40,150)
	         Image and HSV range E.g：(0,0,90)(177,40,150) 
        '''
        H = [];S = [];V = []
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
            cv.putText(img, txt1, (30, 80), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        else:
            cv.putText(img, txt2, (30, 80), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv.putText(img, lowerb, (150, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        cv.putText(img, upperb, (150, 50), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        hsv_range = ((int(H_min), int(S_min), int(V_min)), (int(H_max), int(S_max), int(V_max)))
        return img, hsv_range


class simplePID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.targetpoint = 0
        self.intergral = 0
        self.derivative = 0
        self.prevError = 0

    def compute(self, target, current):
        error = target - current
        self.intergral += error
        self.derivative = error - self.prevError
        self.targetpoint = self.kp * error + self.ki * self.intergral + self.kd * self.derivative
        self.prevError = error
        return self.targetpoint

    def reset(self):
        self.targetpoint = 0
        self.intergral = 0
        self.derivative = 0
        self.prevError = 0
