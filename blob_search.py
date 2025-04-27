#!/usr/bin/env python

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = np.arctan2(127 - 126, 417 - 343)
beta = np.sqrt((375 - 430)**2 + (167 - 220)**2) / 100   # Convert to mm
tx, ty = np.array([(136 - 240) / beta, (405 - 320) / beta]) - np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]) @ np.array([100, 100])

# Function that converts image coord to world coord
def IMG2W(col, row):
    O_r = 240
    O_c = 320

    x_c = (row - O_r) / beta
    y_c = (col - O_c) / beta

    R_cw = np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta), np.cos(theta)]])
    x_w, y_w = R_cw.T @ (np.array([x_c, y_c]) - np.array([tx, ty])) - np.array([4, 8])
    return (x_w, y_w)

# ========================= Student's code ends here ===========================

lowers = {"green": (40, 50, 50), "yellow": (20, 100, 100), "red": (0, 50, 50)}
uppers = {"green": (80, 255, 255), "yellow": (30, 255, 255), "red": (10, 255, 255)}

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False

    # Filter by Area.
    params.minArea = 200
    params.maxArea = 500
    params.filterByArea = True

    # Filter by Circularity
    params.filterByCircularity = False

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================

    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lowers[color], uppers[color])

    # ========================= Student's code ends here ===========================

    keypoints = detector.detect(mask_image)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, None, color=(255, 0, 0), flags=cv2.DRAW_MATCHES_FLAGS_DEFAULT)

    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow("Mask View")
    cv2.imshow("Mask View", mask_image)
    cv2.namedWindow("Keypoint View")
    cv2.imshow("Keypoint View", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
