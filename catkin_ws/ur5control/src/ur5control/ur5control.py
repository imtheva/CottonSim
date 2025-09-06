#!/usr/bin/python3
import rospy
from std_msgs.msg import Float64
import time


def test():
   goal_publisher = rospy.Publisher('/shoulder_lift_joint_position_controller/command', Float64, queue_size=10)
   goal_publisher_2 = rospy.Publisher('/shoulder_pan_joint_position_controller/command', Float64, queue_size=10)
   goal_publisher_3 = rospy.Publisher('/elbow_joint_position_controller/command', Float64, queue_size=10)
   goal_publisher_4 = rospy.Publisher('/wrist_1_joint_position_controller/command', Float64, queue_size=10)
   goal_publisher_5 = rospy.Publisher('/wrist_2_joint_position_controller/command', Float64, queue_size=10)
   goal_publisher_6 = rospy.Publisher('/wrist_3_joint_position_controller/command', Float64, queue_size=10)
   # Starts a new node
   rospy.init_node('ur5_arm_controller', anonymous=True)
   goal_msg = Float64()
   # goal_msg.data = -1.570

   # rospy.sleep(1)

   # goal_publisher.publish(goal_msg)

   # print(goal_msg)


   # goal_msg.data = -3.140

   # rospy.sleep(3)

   # goal_publisher_2.publish(goal_msg)

   # print(goal_msg)


   # goal_msg.data = 1.570

   # rospy.sleep(3)

   # goal_publisher_3.publish(goal_msg)

   # print(goal_msg)

   # goal_msg.data = -3.140

   # rospy.sleep(3)

   # goal_publisher_4.publish(goal_msg)

   # print(goal_msg)


   # goal_msg.data = 1.570

   # rospy.sleep(3)

   # goal_publisher_5.publish(goal_msg)

   # print(goal_msg)


   # goal_msg.data = 3.140

   # rospy.sleep(3)

   # goal_publisher_6.publish(goal_msg)

   # print(goal_msg)


   # # shoulder_lift_joint_position_controller

   # goal_msg.data = 0.0

   # rospy.sleep(1)

   # goal_publisher.publish(goal_msg)

   # print(goal_msg)

   # # shoulder_pan_joint_position_controller
   # goal_msg.data = 0

   # rospy.sleep(3)

   # goal_publisher_2.publish(goal_msg)

   # print(goal_msg)

   # # elbow_joint_position_controller/command
   # goal_msg.data = -2.60

   # rospy.sleep(3)

   # goal_publisher_3.publish(goal_msg)

   # print(goal_msg)

   # # wrist_1_joint_position_controller/command

   # goal_msg.data = 0

   # rospy.sleep(3)

   # goal_publisher_4.publish(goal_msg)

   # print(goal_msg)

   # # wrist_2_joint_position_controller/command
   # goal_msg.data = 1.570

   # rospy.sleep(3)

   # goal_publisher_5.publish(goal_msg)

   # print(goal_msg)

   # # wrist_3_joint_position_controller/command
   # goal_msg.data = 0

   # rospy.sleep(3)

   # goal_publisher_6.publish(goal_msg)

   # print(goal_msg)


# Flat position

   # shoulder_lift_joint_position_controller

   goal_msg.data = -2.094395

   rospy.sleep(1)

   goal_publisher.publish(goal_msg)

   print(goal_msg)

   # shoulder_pan_joint_position_controller / base angle
   goal_msg.data = 3.14159
   rospy.sleep(3)

   goal_publisher_2.publish(goal_msg)

   print(goal_msg)

   # elbow_joint_position_controller/command
   goal_msg.data = 2.094395

   rospy.sleep(3)

   goal_publisher_3.publish(goal_msg)

   print(goal_msg)

   # wrist_1_joint_position_controller/command

   goal_msg.data = 0
   rospy.sleep(3)

   goal_publisher_4.publish(goal_msg)

   print(goal_msg)

   # wrist_2_joint_position_controller/command
   goal_msg.data = 1.5708

   rospy.sleep(3)

   goal_publisher_5.publish(goal_msg)

   print(goal_msg)

   # wrist_3_joint_position_controller/command
   goal_msg.data = 0

   rospy.sleep(3)

   goal_publisher_6.publish(goal_msg)

   print(goal_msg)



#    rospy.spin()


if __name__ == '__main__':
   try:
       #Testing our function
       test()

   except rospy.ROSInterruptException: pass