#!/usr/bin/python3

# Import the necessary packages
import threading
import sys
import cv2
import numpy as np
import math
import time
import logging
sys.path.append('/home/pi/ArmPi/')
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

AK = ArmIK()

# Camera calibration parameters
corners_length = 2.1
#square_length = 3
calibration_size = (7, 7)
save_path = '/home/pi/ArmPi/CameraCalibration/calibration_images/'
calibration_param_path = '/home/pi/ArmPi/CameraCalibration/calibration_param'
map_param_path = '/home/pi/ArmPi/CameraCalibration/map_param'

param_data = np.load(map_param_path + '.npz')

class Camera:
    def __init__(self, resolution=(640, 480)):
        self.cap = None
        self.width = resolution[0]
        self.height = resolution[1]
        self.frame = None
        self.opened = False
        self.param_data = np.load(calibration_param_path + '.npz')
        
        self.mtx = self.param_data['mtx_array']
        self.dist = self.param_data['dist_array']
        self.newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (self.width, self.height), 0, (self.width, self.height))
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.mtx, self.dist, None, self.newcameramtx, (self.width,self.height), 5)
        
        self.th = threading.Thread(target=self.camera_task, args=(), daemon=True)
        self.th.start()

    def camera_open(self):
        try:
            self.cap = cv2.VideoCapture(-1)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_SATURATION, 40)
            self.opened = True
        except Exception as e:
            print('打开摄像头失败:', e)

    def camera_close(self):
        try:
            self.opened = False
            time.sleep(0.2)
            if self.cap is not None:
                self.cap.release()
                time.sleep(0.05)
            self.cap = None
        except Exception as e:
            print('关闭摄像头失败:', e)

    def camera_task(self):
        while True:
            try:
                if self.opened and self.cap.isOpened():
                    ret, frame_tmp = self.cap.read()
                    if ret:
                        frame_resize = cv2.resize(frame_tmp, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
                        self.frame = cv2.remap(frame_resize, self.mapx, self.mapy, cv2.INTER_LINEAR)
                    else:
                        print(1)
                        self.frame = None
                        cap = cv2.VideoCapture(-1)
                        ret, _ = cap.read()
                        if ret:
                            self.cap = cap
                elif self.opened:
                    print(2)
                    cap = cv2.VideoCapture(-1)
                    ret, _ = cap.read()
                    if ret:
                        self.cap = cap              
                else:
                    time.sleep(0.01)
            except Exception as e:
                print('获取摄像头画面出错:', e)
                time.sleep(0.01)


class ColorTracker:
    def __init__(self):
        self.size = (640, 480)
        self.last_x, self.last_y = 0, 0
        self.rect = None
        self.track = False
        self.get_roi = False
        self.unreachable = False
        self.world_X, self.world_Y = 0, 0
        self.world_x, self.world_y = 0, 0
        self.detect_color = 'None'
        self.action_finish = True
        self.rotation_angle = 0
        self.center_list = []
        self.count = 0
        self.start_pick_up = False
        self.first_move = True
        self.start_count_t1 = True
        self.roi = None
        self.__target_color = ['red', 'green', 'blue']
        self.last_x, self.last_y = 0, 0  
        self.color_range = {
            'red': [(0, 151, 100), (255, 255, 255)], 
            'green': [(0, 0, 0), (255, 115, 255)], 
            'blue': [(0, 0, 0), (255, 255, 110)], 
            'black': [(0, 0, 0), (56, 255, 255)], 
            'white': [(193, 0, 0), (255, 250, 255)], 
        }
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }
        self.map_param_ = param_data['map_param']
        self.image_center_distance = 0
        self.logger = self.get_logger()

    def getROI(self, box):
        x_min = min(box[0, 0], box[1, 0], box[2, 0], box[3, 0])
        x_max = max(box[0, 0], box[1, 0], box[2, 0], box[3, 0])
        y_min = min(box[0, 1], box[1, 1], box[2, 1], box[3, 1])
        y_max = max(box[0, 1], box[1, 1], box[2, 1], box[3, 1])

        return (x_min, x_max, y_min, y_max)
    
    def getAreaMaxContour(self, contours):  
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:
                    area_max_contour = c

        return area_max_contour, contour_area_max

    def get_logger(self):
        # Setup logger
        logger = logging.getLogger(__name__)
        logger.setLevel(logging.INFO)
        handler = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(message)s')
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        return logger

    def get_mask_roi(self, frame_gb, roi, size):
        if roi is None:
            x, y, w, h = roi
            mask = np.zeros((size[1], size[0]), dtype=np.uint8)
            mask[y:y+h, x:x+w] = 255
            frame_gb = cv2.bitwise_and(frame_gb, frame_gb, mask=mask)
            return frame_gb
    
    def world2pixel(self, l, size):
        l_ = round(l/self.map_param_, 2)

        l_ = self.leMap(l_, 0, 640, 0, size[0])

        return l_
    
    def leMap(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def convertCoordinate(self, x, y, size):
        x = self.leMap(x, 0, size[0], 0, 640)
        x = x - 320
        x_ = round(x * self.map_param_, 2)

        y = self.leMap(y, 0, size[1], 0, 480)
        y = 240 - y
        y_ = round(y * self.map_param_ + self.image_center_distance, 2)

        return x_, y_
    
    def getMaskROI(self, frame_gb, roi, size):
        if roi is not None:  
            x_min, x_max, y_min, y_max = roi
            x_min -= 10
            x_max += 10
            y_min -= 10
            y_max += 10

            if x_min < 0:
                x_min = 0
            if x_max > size[0]:
                x_max = size[0]
            if y_min < 0:
                y_min = 0
            if y_max > size[1]:
                y_max = size[1]

            black_img = np.zeros([size[1], size[0]], dtype=np.uint8)
            black_img = cv2.cvtColor(black_img, cv2.COLOR_GRAY2RGB)
            black_img[y_min:y_max, x_min:x_max] = frame_gb[y_min:y_max, x_min:x_max]
            
            return black_img
        else:
            # Return original frame_gb if roi is None
            return frame_gb  

    
    def getCenter(self, rect, roi, size, square_length):
        x_min, x_max, y_min, y_max = roi
        if rect[0][0] >= size[0]/2:
            x = x_max 
        else:
            x = x_min
        if rect[0][1] >= size[1]/2:
            y = y_max
        else:
            y = y_min


        square_l = square_length/math.cos(math.pi/4)


        square_l = self.world2pixel(square_l, size)

        dx = abs(math.cos(math.radians(45 - abs(rect[2]))))
        dy = abs(math.sin(math.radians(45 + abs(rect[2]))))
        if rect[0][0] >= size[0] / 2:
            x = round(x - (square_l/2) * dx, 2)
        else:
            x = round(x + (square_l/2) * dx, 2)
        if rect[0][1] >= size[1] / 2:
            y = round(y - (square_l/2) * dy, 2)
        else:
            y = round(y + (square_l/2) * dy, 2)

        return  x, y

    def getAngle(self, x, y, angle):
        theta6 = round(math.degrees(math.atan2(abs(x), abs(y))), 1)
        angle = abs(angle)
        
        if x < 0:
            if y < 0:
                angle1 = -(90 + theta6 - angle)
            else:
                angle1 = theta6 - angle
        else:
            if y < 0:
                angle1 = theta6 + angle
            else:
                angle1 = 90 - theta6 - angle

        if angle1 > 0:
            angle2 = angle1 - 90
        else:
            angle2 = angle1 + 90

        if abs(angle1) < abs(angle2):
            servo_angle = int(500 + round(angle1 * 1000 / 240))
        else:
            servo_angle = int(500 + round(angle2 * 1000 / 240))
        return servo_angle

    def run(self, img, color):
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)

        if self.get_roi:
            self.get_roi = False
            frame_gb = self.getMaskROI(frame_gb, self.roi, self.size)    

        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

        box = []
        # for i in self.color_range:
        #for i in self.__target_color:
            # if i in self.__target_color:
        
        frame_mask = cv2.inRange(frame_lab, self.color_range[color][0], self.color_range[color][1])
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        areaMaxContour, area_max = self.getAreaMaxContour(contours)
        world_x, world_y, angle = None, None, None

        if area_max > 2500:
            rect = cv2.minAreaRect(areaMaxContour)
            box.append(np.int0(cv2.boxPoints(rect)))

            self.roi = self.getROI(box[-1])
            # self.get_roi = True

            img_centerx, img_centery = self.getCenter(rect, self.roi, self.size, square_length)
            world_x, world_y = self.convertCoordinate(img_centerx, img_centery, self.size)
            world_y += 20
            angle = self.getAngle(world_x, world_y, rect[2])

            self.logger.info('Color: %s, World X: %s, World Y: %s', color, world_x, world_y)

            cv2.drawContours(img, [box[-1]], -1, self.range_rgb[color], 2)
            cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[-1][0, 0], box[-1][2, 0]), box[-1][2, 1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[color], 1)
            
            # if i == color:
            #     return img, world_x, (world_y+20), i
            
            # distance = math.sqrt(pow(world_x - self.last_x, 2) + pow(world_y - self.last_y, 2))
            # self.last_x, self.last_y = world_x, world_y  # Update last_x and last_y

        return img, world_x, world_y, angle
    
    def main(self):
        
        # tracker = ColorTracker()

        my_camera = Camera()
        my_camera.camera_open()
        while True:
            img = my_camera.frame
            if img is not None:
                frame = img.copy()
                Frame = self.run(frame)           
                cv2.imshow('Frame', Frame)
                key = cv2.waitKey(1)
                if key == 27:
                    break
        my_camera.camera_close()
        cv2.destroyAllWindows()


if __name__ == '__main__':

    tracker = ColorTracker()
    tracker.main()

    # my_camera = Camera()
    # my_camera.camera_open()
    # while True:
    #     img = my_camera.frame
    #     if img is not None:
    #         frame = img.copy()
    #         Frame = tracker.run(frame)           
    #         cv2.imshow('Frame', Frame)
    #         key = cv2.waitKey(1)
    #         if key == 27:
    #             break
    # my_camera.camera_close()
    # cv2.destroyAllWindows()