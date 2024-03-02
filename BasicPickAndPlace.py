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
    # Initialize Camera and Movement objects
    camera = Camera()
    tracker = ColorTracker()
    movement = Movement()

    color = 'blue'
    tracker.detect_color = ['red', 'green', 'blue']
    #tracker.start()

    # Create task and frame queues
    task_queue = queue.Queue()
    frame_queue = queue.Queue()

    camera.camera_open()
    while True:
            img = camera.frame
            if img is not None:
                print("Camera initialized")
                frame, x, y, i = tracker.run(img)
                movement.pick_and_place(x, y, color)
                #break

    # Start camera and movement threads
    # camera_thread = threading.Thread(target=camera.camera_task, daemon=True)
    # tracker_thread = threading.Thread(target=tracker.main, daemon=True)
    # movement_thread = threading.Thread(target=movement.move, daemon=True)
    #runner_thread = threading.Thread(target=runner(tracker, movement), daemon=True)



    # Start the motion control worker thread
    motion_thread = threading.Thread(target=movement.pick_and_place, args=(task_queue,))
    motion_thread.start()

    # Start the image detection worker thread
    detection_thread = threading.Thread(target=image_detection_worker, args=(camera, tracker, task_queue, frame_queue))
    detection_thread.start()

    print("Starting threads")
    # camera_thread.start()
    # tracker_thread.start()
    # movement_thread.start()
    #runner_thread.start()
    print("Threads started")

    try:
        while True:
            time.sleep(1)  # Keep the main thread alive
    except KeyboardInterrupt:
    #     tracker_thread.join()
    #     camera.camera_close()
    #     camera_thread.join()
    #     movement.exit()
    #     movement_thread.join()
         print("Program exit")
        
# def main():
#     my_camera = Camera()
#     my_camera.camera_open()

#     color = 'red'
#     tracker = ColorTracker()
#     #tracker.set_target_color((color,))
#     tracker.detect_color = color
#     #tracker.start()

#     motion_controller = Movement()

#     while True:

#         img = my_camera.frame
#         if img is not None:
#             frame, x, y, color= tracker.run(img)
#             motion_controller.pick_and_place(x, y, color)
#             cv2.imshow('Frame', frame)
#             key = cv2.waitKey(1)
#             if key == 27:  # ESC key
#                 break

#     my_camera.camera_close()
#     cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
