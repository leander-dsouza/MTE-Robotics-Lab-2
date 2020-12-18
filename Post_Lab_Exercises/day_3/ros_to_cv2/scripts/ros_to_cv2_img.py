#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError


bridge = CvBridge()
frame = None


def blob_detect(image, hsv_min, hsv_max, blur=0, blob_params=None, imshow=False):
    if blur > 0:
        image = cv2.blur(image, (blur, blur))

        if imshow:
            cv2.imshow("Blur", image)
            cv2.waitKey(0)

    # BGR to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # - Apply HSV threshold
    mask = cv2.inRange(hsv, hsv_min, hsv_max)

    # - Show HSV Mask
    if imshow:
        cv2.imshow("HSV Mask", mask)

    # - dilate makes the in range areas larger
    mask = cv2.dilate(mask, None, iterations=2)

    if imshow:
        cv2.imshow("Dilate Mask", mask)
        cv2.waitKey(0)

    mask = cv2.erode(mask, None, iterations=2)

    # - Show dilate/erode mask
    if imshow:
        cv2.imshow("Erode Mask", mask)
        cv2.waitKey(0)

    if blob_params is None:
        # Set up the SimpleBlobdetector with default parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 0;
        params.maxThreshold = 50;

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 5000
        params.maxArea = 500000

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.5

        # Filter by Inertia
        params.filterByInertia = True
        params.minInertiaRatio = 0.5

    else:
        params = blob_params

        # - Apply blob detection
    detector = cv2.SimpleBlobDetector_create(params)

    # Reverse the mask: blobs are black on white
    reversemask = 255 - mask

    if imshow:
        cv2.imshow("Reverse Mask", reversemask)
        cv2.waitKey(0)

    keypoints = detector.detect(reversemask)

    return keypoints, reversemask


def draw_keypoints(image, keypoints, line_color=(255, 0, 255), imshow=True):
    im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), line_color,
                                          cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    if imshow:
        cv2.imshow("Keypoints", im_with_keypoints)

    return (im_with_keypoints)


def draw_window(image, window_adim, color=(255, 0, 0), line=5, imshow=False):
    rows = image.shape[0]
    cols = image.shape[1]

    x_min_px = int(cols * window_adim[0])
    y_min_px = int(rows * window_adim[1])
    x_max_px = int(cols * window_adim[2])
    y_max_px = int(rows * window_adim[3])

    # -- Draw a rectangle from top left to bottom right corner
    image = cv2.rectangle(image, (x_min_px, y_min_px), (x_max_px, y_max_px), color, line)

    if imshow:
        cv2.imshow("Keypoints", image)

    return (image)


def apply_search_window(image, window_adim=[0.0, 0.0, 1.0, 1.0]):
    rows = image.shape[0]
    cols = image.shape[1]
    x_min_px = int(cols * window_adim[0])
    y_min_px = int(rows * window_adim[1])
    x_max_px = int(cols * window_adim[2])
    y_max_px = int(rows * window_adim[3])

    # --- Initialize the mask as a black image
    mask = np.zeros(image.shape, np.uint8)

    # --- Copy the pixels from the original image corresponding to the window
    mask[y_min_px:y_max_px, x_min_px:x_max_px] = image[y_min_px:y_max_px, x_min_px:x_max_px]

    # --- return the mask
    return (mask)


def get_blob_relative_position(image, keyPoint):
    rows = float(image.shape[0])
    cols = float(image.shape[1])
    # print(rows, cols)
    center_x = 0.5 * cols
    center_y = 0.5 * rows
    # print(center_x)
    x = (keyPoint.pt[0] - center_x) / (center_x)
    y = (keyPoint.pt[1] - center_y) / (center_y)
    return (x, y)



def image_callback(ros_image):
    global frame
    try:
        frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        cv2.imshow("image", frame)
    except CvBridgeError as e:
        print(e)

    lower_color = (36, 25, 25)
    higher_color = (70, 255, 255)

    string1 = "first frame"
    string2 = ""
    string3 = ""
    window = [0, 0, 1, 1]

    keypoints, _ = blob_detect(frame, lower_color, higher_color, blur=3, blob_params=None, imshow=False)
    final_image = draw_keypoints(frame, keypoints, imshow=True)
    cv2.putText(final_image, string1, (25, 25), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 1)
    cv2.putText(final_image, string2, (25, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 1)
    cv2.putText(final_image, string3, (25, 75), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 1)
    cv2.imshow("final image", final_image)
    for i, keyPoint in enumerate(keypoints):
        x = keyPoint.pt[0]
        y = keyPoint.pt[1]
        s = keyPoint.size
        print("kp %d: s = %3d   x = %3d  y= %3d" % (i, s, x, y))

        x, y = get_blob_relative_position(frame, keyPoint)
        print(" x = %3d  y= %3d" % (x, y))
        string1 = 'x=' + str(x)
        string2 = 'y=' + str(y)
        string3 = 's=' + str(s)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        return






def listener():
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

    rospy.spin()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    try:
        rospy.init_node('ar_tracking_node', anonymous=True, disable_signals=True)
        rate = rospy.Rate(100)
        listener()

    except rospy.ROSInterruptException:
        pass