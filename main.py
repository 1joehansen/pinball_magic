import os
import time
import sys

import cv2
import mss as mss
import numpy
from pynput.keyboard import Key, Controller

if __name__ == "__main__":

    print("Launching Pinball...")
    # os.system(r"start C:\Users\1joeh\Desktop\Pinball.lnk")
    #
    # print("Game initializing...")
    # time.sleep(15)
    #
    # keyboard = Controller()
    # keyboard.press(Key.space)
    # time.sleep(1.1)
    # keyboard.release(Key.space)


    # Set up tracker.
    # Instead of MIL, you can also use

    tracker_types = ['BOOSTING', 'MIL', 'KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
    tracker_type = "CSRT"

    if tracker_type == 'BOOSTING':
        tracker = cv2.TrackerBoosting_create()
    if tracker_type == 'MIL':
        tracker = cv2.TrackerMIL_create()
    if tracker_type == 'KCF':
        tracker = cv2.TrackerKCF_create()
    if tracker_type == 'TLD':
        tracker = cv2.TrackerTLD_create()
    if tracker_type == 'MEDIANFLOW':
        tracker = cv2.TrackerMedianFlow_create()
    if tracker_type == 'GOTURN':
        tracker = cv2.TrackerGOTURN_create()
    if tracker_type == 'MOSSE':
        tracker = cv2.TrackerMOSSE_create()
    if tracker_type == "CSRT":
        tracker = cv2.TrackerCSRT_create()

    sct = mss.mss()
    # Part of the screen to capture
    monitor = {'top': 274, 'left': 590, 'width': 752, 'height': 522}

    # Get raw pixels from the screen, save it to a Numpy array
    img = numpy.array(sct.grab(monitor))
    img = img[:, :, 0:3]
    frame = img

    use_video = True   # select video or screen capture mode

    if use_video:
        # Read video
        video = cv2.VideoCapture(r"C:\Users\1joeh\Videos\pinball_track_test.wmv")

        # Exit if video not opened.
        if not video.isOpened():
            print("Could not open video")
            sys.exit()

        # Read first frame.
        ok, frame = video.read()

        if not ok:
            print('Cannot read video file')
            sys.exit()
    else:
        frame = img
        cv2.imshow("Test 1", img)

    # Uncomment the line below to select a different bounding box
    # bbox = cv2.selectROI(frame, False)

    # Initialize tracker with first frame and bounding box
    # ok = tracker.init(frame, bbox)

    frame_old = frame

    while True:

        if use_video:
            # Read a new frame
            ok, frame = video.read()
            if not ok:
                break

            # cv2.imshow("Test 2", frame)
        else:
            # Get raw pixels from the screen, save it to a Numpy array
            img = numpy.array(sct.grab(monitor))
            img = img[:, :, 0:3]
            img = numpy.array(img)
            frame = img
            # cv2.imshow("Test 2", img)

        # Start timer
        timer = cv2.getTickCount()

        # Update tracker
        # ok, bbox = tracker.update(frame)

        # Calculate Frames per second (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);

        # Draw bounding box
        # if ok:
        #     # Tracking success
        #     p1 = (int(bbox[0]), int(bbox[1]))
        #     p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        #     cv2.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
        # else:
        #     # Tracking failure
        #     cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255),
        #                 2)
        #
        # # Display tracker type on frame
        # cv2.putText(frame, tracker_type + " Tracker", (100, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);
        #
        # # Display FPS on frame
        # cv2.putText(frame, "FPS : " + str(int(fps)), (100, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

        # Display result
        # cv2.imshow("Tracking", numpy.dot((frame - frame_old), [0.299, 0.587, 0.114])) # grayscale

        # contrast application
        alpha = 5
        beta = 0

        display = abs(frame_old - frame)

        display = alpha * display + beta

        cv2.imshow("Tracking", display )
        time.sleep(.1)
        frame_old = frame

        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27: break









    # while True:
    #
    #     keyboard.press('/')
    #     time.sleep(0.1)
    #     keyboard.release('/')
    #     time.sleep(0.5)
    #
    #     keyboard.press('z')
    #     time.sleep(0.1)
    #     keyboard.release('z')
    #     time.sleep(0.5)

