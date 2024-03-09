#!/usr/bin/python3

import threading
import time
import sys
import queue
import cv2
sys.path.append('/home/matt/PiArm/')  # Update this path to the directory containing your programs

from BasicColorTracking import Camera, ColorTracker
from BasicMovement import Movement

def motionworker(taskqueue):
    while True:
        task = taskqueue.get()
        if task is None:  # Sentinel value to stop the thread
            break
        x, y, color = task
        #print("arm")
        print(f"Motion Worker Task Var: {x}, {y}, {color}")
        # print(detected)
        # if detected:
        #     # motioncontroller.pick_and_place(x, y, color)
        #     print(detected)

def image_detection_worker(camera, tracker, color, task_queue, frame_queue):
    while True:
        img = camera.frame
        if img is not None:
            frame, detected, x, y = tracker.run(img)
            #print(x,y)
            task_queue.put((x, y, color))  # Send task to motion worker
            # print(detected)
            frame_queue.put(frame)  # Send frame to main thread for display

def runner(tracker, movement):
    while True:
        movement.world_x = tracker.world_x 
        movement.world_y = tracker.world_y 
        movement.world_X = tracker.world_X
        movement.world_Y = tracker.world_Y

        print("World X: ", movement.world_X)
        print("World Y: ", movement.world_Y)

        movement.move()

def main():
    #Set true if you want to palletize instead of sort the boxes
    palletize = False

    # Initialize Camera and Movement objects
    camera = Camera()
    tracker = ColorTracker()
    movement = Movement()

    #color = 'blue'
    color = ['red', 'green', 'blue']
    tracker.detect_color = ['red', 'green', 'blue']
    #tracker.start()

    # Create task and frame queues
    task_queue = queue.Queue()
    frame_queue = queue.Queue()

    camera.camera_open()
    while True:
            img = camera.frame
            if img is not None:
                num = 0
                for i in color:
                    #print("Camera initialized")
                    frame, x, y, angle = tracker.run(img, i)
                    if x is not None and y is not None and angle is not None:
                        #print(f"X: {x}, Y: {y}, Color: {ret_color}")
                        if palletize:
                            movement.pallet(x, y, angle, num)
                            num += 1
                        else: 
                            movement.pick_and_place(x, y, angle, i)

                
if __name__ == "__main__":
    main()
