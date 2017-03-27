# import the necessary packages
from collections import deque
#from picamera.array import PiRGBArray
#from picamera import PiCamera
import time
import numpy as np
import argparse
import imutils
import cv2

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
        help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
        help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
greenLower = (29, 86, 6)
greenUpper = (64, 255, 255)
orangeLower = (10, 50, 200)
orangeUpper = (100, 200, 255)
greenPts = deque(maxlen=args["buffer"])
orangePts = deque(maxlen=args["buffer"])

def findColouredObject(frame, hsv, lower, upper, pts):

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
                (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

    # update the points queue
    pts.appendleft(center)

def drawTrail(pts):
    # loop over the set of tracked points
    for i in range(1, len(pts)):
        # if either of the tracked points are None, ignore
        # them
        if pts[i - 1] is None or pts[i] is None:
            continue

        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
        cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

def handleFrame(frame):
    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(frame, width=600)
    # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    findColouredObject(frame, hsv, greenLower, greenUpper, greenPts)

    drawTrail(greenPts)

    findColouredObject(frame, hsv, orangeLower, orangeUpper, orangePts)

    drawTrail(orangePts)

        # show the frame to our screen
    cv2.imshow("Frame", frame)

# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
    #init camera
    camera = PiCamera()
    camera.resolution = (640,480)
    camera.framerate = 60
    camera.exposure_mode = 'sports'
    rawCapture = PiRGBArray(camera, size=(640,480))

    #camera warm up
    time.sleep(2.5)

# otherwise, grab a reference to the video file
else:
    camera = cv2.VideoCapture(args["video"])

if not args.get("video", False):
    # keep looping
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        frame = frame.array
        rawCapture.truncate(0)

        handleFrame(frame)

        key = cv2.waitKey(1) & 0xFF
        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break
else:
    # keep looping
    while True:
        (grabbed, frame) = camera.read()
        if not grabbed:
            break
        handleFrame(frame)

        key = cv2.waitKey(1) & 0xFF
        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break

        time.sleep(.01)
# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()

