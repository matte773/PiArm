#!/usr/bin/python3
# coding=utf8

#from BasicColorTracking import *
import sys
sys.path.append('./ArmPi/')
# import cv2
import time
# import Camera
# import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

AK = ArmIK()

class Movement:
    def __init__(self):
        # self.color = BasicColorTracking()
        # self.color.start()
        # self.color.join()
        # #self.color = self.color.color
        self.track = False
        self._stop = False
        self.get_roi = False
        self.center_list = []
        self.first_move = True
        self.__isRunning = False
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True
        self.rect = None
        self.size = (640, 480)
        self.rotation_angle = 0
        self.unreachable = False
        self.world_X, self.world_Y = 0, 0
        self.world_x, self.world_y = 0, 0
        self.servo1 = 500
        self.SERVO_GRIPPER_OPEN = 500
        self.SERVO_GRIPPER_CLOSED = 200
        self.SERVO_BASE_ANGLE = 500
        self.coordinates = {
            'red':   (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5,  1.5),
            'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
            'present':  (-15 + 0.5, 0 - 0.5,  1.5),
        }

        self.initMove()

    def initMove(self):
        Board.setBusServoPulse(1, self.servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def operate_gripper(self, open_gripper):
        if open_gripper:
            Board.setBusServoPulse(1, self.SERVO_GRIPPER_OPEN, 500)
        else:
            Board.setBusServoPulse(1, self.SERVO_GRIPPER_CLOSED, 500)
        time.sleep(1)

    def rotate_gripper(self, angle):
        Board.setBusServoPulse(2, angle, 500)
        time.sleep(1)

    def move_to(self, x, y, z, pitch=-90, roll=-90, yaw=0, duration=1000):
        AK.setPitchRangeMoving((x, y, z), pitch, roll, yaw, duration)
        time.sleep(duration / 1000)

    def pick_from(self, x, y, angle):
        self.operate_gripper(False)  # Ensure gripper is open
        print(f"Picking from ({x}, {y}) and Gripper open")
        self.move_to(x, y, 12)  # Move above the object
        self.rotate_gripper(angle)  # Rotate the gripper to the object
        self.move_to(x, y, 2)  # Move down to the object
        self.operate_gripper(True)  # Close gripper to pick
        self.move_to(x, y, 12)  # Lift the object
        print("Pick Up Done")


    def place_at(self, location):
        target_x, target_y, target_z = self.coordinates[location]
        print(f"Placing at ({target_x}, {target_y}, {target_z})")
        self.move_to(target_x, target_y, 12)  # Move above the place location
        self.move_to(target_x, target_y, target_z)  # Move down to the place position
        self.operate_gripper(False)  # Open gripper to place the object
        self.move_to(target_x, target_y, 12)  # Move back up
        
        return True

    def pick_and_place(self, x, y, angle, color):
        self.pick_from(x, y, angle)
        self.place_at(color)

    def setBuzzer(self, timer):
        Board.setBuzzer(0)
        Board.setBuzzer(1)
        time.sleep(timer)
        Board.setBuzzer(0)

    # def set_rgb(self, color):
        if color == "red":
            Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
            Board.RGB.show()
        elif color == "green":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
            Board.RGB.show()
        elif color == "blue":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
            Board.RGB.show()
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
            Board.RGB.show()


if __name__ == '__main__':
    print("Movement class is running...")
