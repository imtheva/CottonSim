#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import cv2
import torch
import time
import message_filters
import numpy as np

from ultralytics import YOLO

from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

from std_msgs.msg import Int16,String

from actionlib_msgs.msg import GoalStatusArray


model = YOLO('path')  # load a model

def image_callback(pCam):
    try:
        # Convert ROS Image message to OpenCV image using CVBridge
        pCam_cv_image = bridge.imgmsg_to_cv2(pCam, "bgr8")
        # sCam_cv_image = bridge.imgmsg_to_cv2(sCam, "bgr8")


    except CvBridgeError as e:
        rospy.logerr(e)
        return

    pubminx = rospy.Publisher('mask/ground/minx',Int16,queue_size=10)
    pubminy = rospy.Publisher('mask/ground/miny',Int16,queue_size=10)
    pubmaxx = rospy.Publisher('mask/ground/maxx',Int16,queue_size=10)
    pubmaxy = rospy.Publisher('mask/ground/maxy',Int16,queue_size=10)

    pubmidwextendf = rospy.Publisher('mask/ground/midwextendf',Int16,queue_size=10)
    pubmidwextendh = rospy.Publisher('mask/ground/midwextendh',Int16,queue_size=10)
    pubgMaskLCenter = rospy.Publisher('mask/ground/gMaskLCenter',Int16,queue_size=10)
    pubgMaskHCenter = rospy.Publisher('mask/ground/gMaskHCenter',Int16,queue_size=10)


    resultsG = model.predict(source=pCam_cv_image,show=False,classes=[1],show_labels=False, boxes=False,retina_masks=True)  # predict on an image


    # Ground Layer based mask points
    masksG = resultsG[0].masks
    pointsG = masksG[0].xy


    flattened_data = [point for sublist in pointsG for point in sublist]

    # Extract x and y coordinates into separate lists
    x_coordinates = [point[0] for point in flattened_data]
    y_coordinates = [point[1] for point in flattened_data]

    # Find the maximum and minimum x, y coordinates
    max_x = max(x_coordinates)
    min_x = min(x_coordinates)
    max_y = max(y_coordinates)
    min_y = min(y_coordinates)


    # Find the respective points with maximum and minimum x, y coordinates
    max_x_points = [point for point in flattened_data if point[0] == max_x]
    min_x_points = [point for point in flattened_data if point[0] == min_x]
    max_y_points = [point for point in flattened_data if point[1] == max_y]
    min_y_points = [point for point in flattened_data if point[1] == min_y]

    # respect joints
    resp_ymax_points = [point[1] for point in max_x_points]
    resp_ymin_points = [point[1] for point in min_x_points]
    resp_xmax_points = [point[0] for point in max_y_points]
    resp_xmin_points = [point[0] for point in min_y_points]

    pubminx.publish(int(min_x))
    pubmaxx.publish(int(max_x))
    pubminy.publish(int(min_y))
    pubmaxy.publish(int(max_y))


    # Ground Mask Layer points for creating line
    gMaskLCenter = ((min(resp_xmin_points)+max(resp_xmin_points))/2)
    # print("gMaskLCenter: ",gMaskLCenter)

    gMaskHCenter = ((min(resp_xmax_points)+max(resp_xmax_points))/2)
    # print("gMaskHCenter: ",gMaskHCenter)

    midw = int(gMaskLCenter)
    midh = int(min_y)
    midwextendf = midw
    midwextendh = midh - 80

    pubmidwextendf.publish(int(midwextendf ))
    pubmidwextendh.publish(int(midwextendh))
    pubgMaskLCenter.publish(int(gMaskLCenter ))
    pubgMaskHCenter.publish(int(gMaskHCenter))



if __name__ == '__main__':
    rospy.init_node('groundPublisher')


    bridge = CvBridge()

    rospy.sleep(1)


    rospy.Subscriber('/realsense_p/color/image_raw', Image, image_callback)


    rospy.spin()

    cv2.destroyAllWindows()