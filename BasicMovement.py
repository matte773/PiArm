#!/usr/bin/python3
# coding=utf8

from BasicColorTracking import *
import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
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
        }

        self.initMove()

    def start(self):
        self.reset()
        self.__isRunning = True
        print("ColorTracking Start")

    def stop(self):
        self._stop = True
        self.__isRunning = False
        print("ColorTracking Stop")

    def exit(self):
        self._stop = True
        self.__isRunning = False
        print("ColorTracking Exit")

    def reset(self):
        self.count = 0
        self._stop = False
        self.track = False
        self.get_roi = False
        self.center_list = []
        self.first_move = True
        self.__target_color = ()
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True

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


    def place_at(self, color):
        target_x, target_y, target_z = self.coordinates[color]
        print(f"Placing at ({target_x}, {target_y}, {target_z})")
        self.move_to(target_x, target_y, 12)  # Move above the place location
        self.move_to(target_x, target_y, target_z)  # Move down to the place position
        self.operate_gripper(False)  # Open gripper to place the object
        self.move_to(target_x, target_y, 12)  # Move back up
        
        return True
    
    def place_at_pallet(self, number):

        print(f"Number: {number}")

        target_x, target_y = -15 + 1, -7 - 0.5

        if number == 1:
            target_z = 5
        elif number == 2:
            target_z = 8
        else:
            target_z = 1.5

        print(f"Placing at ({target_x}, {target_y}, {target_z})")
        self.move_to(target_x, target_y, 12)  # Move above the place location
        self.move_to(target_x, target_y, target_z)  # Move down to the place position
        self.operate_gripper(False)  # Open gripper to place the object
        self.move_to(target_x, target_y, 12)  # Move back up
        
        return True

    def pick_and_place(self, x, y, angle, color):
        self.pick_from(x, y, angle)
        self.place_at(color)

    def pallet(self, x, y, angle, number):
        self.pick_from(x, y, angle)
        self.place_at_pallet(number)


    def move(self, x, y):
        coordinate = {
            'red':   (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5,  1.5),
            'blue':  (-15 + 0.5, 0 - 0.5,  1.5),
        }
        self.world_X = x
        self.world_Y = y
        self.world_x = x
        self.world_y = y

        while True:
            if self.__isRunning:
                if self.first_move and self.start_pick_up:                
                    action_finish = False
                    self.set_rgb(self.detect_color)
                    self.setBuzzer(0.1)               
                    result = AK.setPitchRangeMoving((self.world_X, self.world_Y - 2, 5), -90, -90, 0) # 不填运行时间参数，自适应运行时间
                    if result == False:
                        unreachable = True
                    else:
                        unreachable = False
                    time.sleep(result[2]/1000) # 返回参数的第三项为时间
                    self.start_pick_up = False
                    first_move = False
                    action_finish = True
                elif not self.first_move and not self.unreachable: # 不是第一次检测到物体
                    self.set_rgb(self.detect_color)
                    if track: # 如果是跟踪阶段
                        if not self.__isRunning: # 停止以及退出标志位检测
                            continue
                        AK.setPitchRangeMoving((self.world_x, self.world_y - 2, 5), -90, -90, 0, 20)
                        time.sleep(0.02)                    
                        track = False
                    if self.start_pick_up: #如果物体没有移动一段时间，开始夹取
                        action_finish = False
                        if not self.__isRunning: # 停止以及退出标志位检测
                            continue
                        Board.setBusServoPulse(1, self.servo1 - 280, 500)  # 爪子张开
                        # 计算夹持器需要旋转的角度
                        servo2_angle = self.getAngle(self.world_X, self.world_Y, self.rotation_angle)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.8)
                        
                        if not self.__isRunning:
                            continue
                        AK.setPitchRangeMoving((self.world_X, self.world_Y, 2), -90, -90, 0, 1000)  # 降低高度
                        time.sleep(2)
                        
                        if not self.__isRunning:
                            continue
                        Board.setBusServoPulse(1, self.servo1, 500)  # 夹持器闭合
                        time.sleep(1)
                        
                        if not self.__isRunning:
                            continue
                        Board.setBusServoPulse(2, 500, 500)
                        AK.setPitchRangeMoving((self.world_X, self.world_Y, 12), -90, -90, 0, 1000)  # 机械臂抬起
                        time.sleep(1)
                        
                        if not self.__isRunning:
                            continue
                        # 对不同颜色方块进行分类放置
                        result = AK.setPitchRangeMoving((coordinate[self.detect_color][0], coordinate[self.detect_color][1], 12), -90, -90, 0)   
                        time.sleep(result[2]/1000)
                        
                        if not self.__isRunning:
                            continue
                        servo2_angle = self.getAngle(coordinate[self.detect_color][0], coordinate[self.detect_color][1], -90)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.5)

                        if not self.__isRunning:
                            continue
                        AK.setPitchRangeMoving((coordinate[self.detect_color][0], coordinate[self.detect_color][1], coordinate[self.detect_color][2] + 3), -90, -90, 0, 500)
                        time.sleep(0.5)
                        
                        if not self.__isRunning:
                            continue
                        AK.setPitchRangeMoving((coordinate[self.detect_color]), -90, -90, 0, 1000)
                        time.sleep(0.8)
                        
                        if not self.__isRunning:
                            continue
                        Board.setBusServoPulse(1, self.servo1 - 200, 500)  # 爪子张开，放下物体
                        time.sleep(0.8)
                        
                        if not self.__isRunning:
                            continue                    
                        AK.setPitchRangeMoving((coordinate[self.detect_color][0], coordinate[self.detect_color][1], 12), -90, -90, 0, 800)
                        time.sleep(0.8)

                        self.initMove()  
                        time.sleep(1.5)

                        self.detect_color = 'None'
                        self.first_move = True
                        self.get_roi = False
                        self.action_finish = True
                        self.start_pick_up = False
                        self.set_rgb(self.detect_color)
                    else:
                        time.sleep(0.01)
            else:
                if self._stop:
                    self._stop = False
                    Board.setBusServoPulse(1, self.servo1 - 70, 300)
                    time.sleep(0.5)
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                    time.sleep(1.5)
                time.sleep(0.01)

    def initMove(self):
        Board.setBusServoPulse(1, self.servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def setBuzzer(self, timer):
        Board.setBuzzer(0)
        Board.setBuzzer(1)
        time.sleep(timer)
        Board.setBuzzer(0)

    def set_rgb(self, color):
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
    # movement = Movement()  # Instantiate the Movement class

    # try:
    #     movement.start()  # Start the movement

    #     # Create a new thread to execute the move method
    #     # move_thread = threading.Thread(target=movement.move)
    #     # move_thread.start()

    #     while True:
    #         # Assuming you have some way to get the x, y coordinates from user input or elsewhere
    #         x = float(input("Enter the x coordinate: "))
    #         y = float(input("Enter the y coordinate: "))
            
    #         # # Set the coordinates and initiate the movement
    #         movement.world_X = x
    #         movement.world_Y = y
    #         movement.first_move = True
    #         movement.detect_color = 'blue'
    #         movement.__target_color = 'blue'
    #         movement.__isRunning = True  # Set the flag to start the movement
    #         movement.start_pick_up = True  # Set the flag to start the pick up

    #         movement.pick_and_place(x, y+20, movement.__target_color)
    #         #movement.move_to(x, y+20, 1.5)
    #         #movement.move(x,y)
    #         # movement.pick_from(x, y+20)
    #         # movement.place_at(movement.__target_color)

    #         # Print statements for debugging
    #         # print("Pre-Move")
    #         # # movement.move()  # Remove this line, it's not needed here.
    #         # print("Post-Move")

    #         # For demonstration purposes, let's assume we just print the coordinates for now
    #         print(f"Moving to coordinates: ({x}, {y})")

    # except KeyboardInterrupt:
    #     print("\nStopping the movement...")
    #     movement.stop()  # Stop the movement if KeyboardInterrupt is raised


# if __name__ == '__main__':
#     movement = Movement()  # Instantiate the Movement class

#     try:
#         movement.start()  # Start the movement
#         while True:
#             # Assuming you have some way to get the x, y coordinates from user input or elsewhere
#             x = float(input("Enter the x coordinate: "))
#             y = float(input("Enter the y coordinate: "))
            
#             # Perform the movement to the specified (x, y) coordinate
#             # Your logic for moving to the specified coordinate goes here
#             # For instance, you might have a method in the Movement class to move to a specific (x, y) coordinate
#             # movement.world_X = x
#             # movement.world_Y = y
#             # movement.first_move = True
#             # movement.detect_color = 'red'
#             print("Pre-Move")
#             # movement.move()
#             print("Post-Move")
#             # You need to implement the move_to_coordinate method in the Movement class

#             # For demonstration purposes, let's assume we just print the coordinates for now
#             print(f"Moving to coordinates: ({x}, {y})")

#     except KeyboardInterrupt:
#         print("\nStopping the movement...")
#         movement.stop()  # Stop the movement if KeyboardInterrupt is raised