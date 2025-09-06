#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import cv2
import torch

from ultralytics import YOLO

# Load a model
model = YOLO('path')  # load the model



def seg_callback(msg):
    try:
        # Convert ROS Image message to OpenCV image using CVBridge
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    except CvBridgeError as e:
        rospy.logerr(e)
        return

    # results =

    results = model.predict(source=cv_image)  # predict on an image
    # cv2.imshow("Segmentation", cv_image)

    # Retrieve the annotated frame
    annotated_frame = results[0].plot(labels=False,boxes=False)

    # Display the annotated frame in a named OpenCV window
    cv2.imshow("Primary Camera", annotated_frame)



    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('Realsense_camera_viewer')

    # Initialize CvBridge: bridging ros msg and opencv
    bridge = CvBridge()

    rospy.sleep(1)

    # Create a subscriber to receive the Realsense camera images
    rospy.Subscriber('/realsense_p/color/image_raw', Image, seg_callback)

    # Spin until the node is shut down
    rospy.spin()

    # Close OpenCV windows
    cv2.destroyAllWindows()
