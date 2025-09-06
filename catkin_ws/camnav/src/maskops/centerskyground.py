#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import cv2
import torch
import time
import message_filters
import numpy as np

from std_msgs.msg import Int16

from ultralytics import YOLO

from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

from actionlib_msgs.msg import GoalStatusArray

def find_intersection(line1, line2):

    (x1, y1), (x2, y2) = line1
    (x3, y3), (x4, y4) = line2

    # Calculate determinants
    det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if det == 0:
        # Lines are parallel or coincident
        return None

    # Calculate the intersection point
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / det
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / det

    return (int(px), int(py))

def LineExtender(point1, point2):

    direction_vector = np.array([point2[0] - point1[0], point2[1] - point1[1]])
    extended_length = 200
    extended_point1 = (int(point1[0] - extended_length * direction_vector[0]),
                       int(point1[1] - extended_length * direction_vector[1]))
    extended_point2 = (int(point2[0] + extended_length * direction_vector[0]),
                       int(point2[1] + extended_length * direction_vector[1]))

    return extended_point1, extended_point2

class Process:
    def __init__(self):
        self.bridge = CvBridge()
        self.gMaskLCenter = 0
        self.gMaskHCenter = 0
        self.groundminy = 0
        self.groundmaxy = 0

        self.res=0

        self.skyrespminx =0
        self.skyrespmaxy =0
        self.skyminy =0
        self.skymaxy =0
        self.skymaxx =0
        self.sMaskHCenter  =0
        self.sMaskLCenter =0


        self.midwextendf = 0


        rospy.Subscriber('mask/ground/gMaskLCenter', Int16, self.gMaskLCenter_callback)
        rospy.Subscriber('mask/ground/gMaskHCenter', Int16, self.gMaskHCenter_callback)
        rospy.Subscriber('mask/ground/miny', Int16, self.groundminy_callback)
        rospy.Subscriber('mask/ground/maxy', Int16, self.groundmaxy_callback)
        rospy.Subscriber('/realsense_p/color/image_raw', Image, self.cam_callback)


        rospy.Subscriber('mask/sky/miny',Int16,self.skyminy_callback)
        rospy.Subscriber('mask/sky/maxy',Int16,self.skymaxy_callback)


        rospy.Subscriber('mask/sky/sMaskLCenter',Int16,self.sMaskLCenter_callback)
        rospy.Subscriber('mask/sky/sMaskHCenter',Int16,self.sMaskHCenter_callback)

        # Publisher for intersection points
        self.pubintersection = rospy.Publisher('mask/intersection/centerskyground', Int16, queue_size=10)

    def gMaskLCenter_callback(self, msg):
        self.gMaskLCenter = msg.data

    def gMaskHCenter_callback(self, msg):
        self.gMaskHCenter = msg.data

    def groundminy_callback(self, msg):
        self.groundminy = msg.data

    def groundmaxy_callback(self, msg):
        self.groundmaxy = msg.data



    def skyminy_callback(self,msg):
        self.skyminy = msg.data

    def skymaxy_callback(self,msg):
        self.skymaxy = msg.data

    def sMaskLCenter_callback(self,msg):
        self.sMaskLCenter = msg.data

    def sMaskHCenter_callback(self,msg):
        self.sMaskHCenter = msg.data


    def cam_callback(self, pCam):
        try:
            self.pCam_cv_image = self.bridge.imgmsg_to_cv2(pCam, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        self.image_callback()

    def image_callback(self):
        # Ground Mask Layer points
        gpointc1 = (self.gMaskLCenter, self.groundminy)
        gpointc2 = (self.gMaskHCenter, self.groundmaxy)

        # Sky Mask Layer points
        skpoint1 = (self.sMaskLCenter, self.skyminy)
        skpoint2 = (self.sMaskHCenter, self.skymaxy)

        # Extend the lines
        gexPoint1, gexPoint2 = LineExtender(gpointc1, gpointc2)
        skextended_point1, skextended_point2 = LineExtender(skpoint1, skpoint2)

        # Draw lines on the image for visualization
        cv2.line(self.pCam_cv_image, gexPoint1, gexPoint2, (32, 50, 1), 3)
        cv2.line(self.pCam_cv_image, skextended_point1, skextended_point2, (255, 0, 0), 3)

        # Find intersection
        intersection = find_intersection((gexPoint1, gexPoint2), (skextended_point1, skextended_point2))
        if intersection:
            print(f"Intersection point: {intersection}")
            cv2.circle(self.pCam_cv_image, intersection, 5, (0, 0, 255), -1)

            # Publish intersection point if within bounds
            if 0 <= intersection[0] <= 640:
                self.pubintersection.publish(intersection[0])
            else:
                print("Intersection point out of bounds, publishing default value.")
                self.pubintersection.publish(300)

        # # Show the result
        cv2.imshow("Intersection", self.pCam_cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('line_intersection')
    process_instance = Process()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
