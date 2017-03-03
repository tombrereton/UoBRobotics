# import packages
import numpy as np
import argparse
import cv2

# construct argument parse and parse the arguments
ap = argparse.Argument()
ap.add_argument("-v", "--videio",
        help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
        help="max buffer size")
args = vars(ap.parse_args())

# if video path was not  passed, use camera
if not args.get("video", False):
    camera = cv2.VideoCapture(0)

# otherwise, grab a reference to the video file
else:
    camera = cv2.VideoCapture(args["video"])

# define colour boundaries
# R >= 100, B >= 15, G >= 17 and R <=200, B<= 56, G <= 50
boundaries = [
        ([17,15,100],[50,56,200]),
        ([86,31,4],[220,88,50]),
        ([25,31,4],[220,88,50]),
        ([103,86,65],[145,133,128])
        )


while True:
    (grabbed, frame) = camera.read()

    # break if we have reached end of video
    if args.get("video") and not grabbed:
        break

    for (lower, upper) in boundaries:

        # convert to numpy arrays
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

        # find the colours within the specified boundaries
        mask = cv2.inRange(frame, lower, upper)
        output = cv2.bitwise_and(frame, frame, mask = mask)

        # show the video




