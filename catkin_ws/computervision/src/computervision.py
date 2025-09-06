#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16, Float64MultiArray


class Process:
    def __init__(self):
        self.res = 0
        self.groundmaxy = 0
        self.groundminy = 0
        self.midwextendf = 0
        self.linearx = 0
        self.angularz = 0

        rospy.Subscriber('mask/cotton/count', Int16, self.res_callback)
        rospy.Subscriber('mask/ground/miny', Int16, self.groundminy_callback)
        rospy.Subscriber('mask/ground/maxy', Int16, self.groundmaxy_callback)
        rospy.Subscriber('mask/intersection/centerskyground', Int16, self.midwextendf_callback)
        self.coordinates_pub = rospy.Publisher("husky/cv/coords", Float64MultiArray, queue_size=10)

    def res_callback(self, msg):
        self.res = msg.data

    def groundminy_callback(self, msg):
        self.groundminy = msg.data

    def groundmaxy_callback(self, msg):
        self.groundmaxy = msg.data

    def midwextendf_callback(self, msg):
        self.midwextendf = msg.data

    def image_callback(self):
        msg = Float64MultiArray()

        if 280 < self.midwextendf <= 360:
            rospy.loginfo("Going Straight...............")
            self.linearx = 0.01
            self.angularz = 0

        elif self.midwextendf > 360:
            rospy.loginfo("Turning right...............")
            self.linearx = 0.01
            self.angularz = -0.01

        elif self.midwextendf <= 280:
            rospy.loginfo("Turning left...............")
            self.linearx = 0.01
            self.angularz = 0.01

        # Publish the updated values
        msg.data = [self.linearx, self.angularz]
        self.coordinates_pub.publish(msg)
        rospy.loginfo(msg.data)

    def run(self):
        rate = rospy.Rate(10)  # Run at 10 Hz
        while not rospy.is_shutdown():
            self.image_callback()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('computervisionguider')
    process = Process()
    process.run()
