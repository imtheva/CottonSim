#!/usr/bin/env python3

import rospy
import threading
from std_msgs.msg import Int16, Float64MultiArray
from geometry_msgs.msg import PoseStamped, Twist
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler




class Process:
    def __init__(self):
        self.coords = [0, 0]
        self.statusin = 0
        self.current_position = None
        self.status_lock = threading.Lock()
        self.status_condition = threading.Condition(self.status_lock)

        # Subscribers
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        rospy.Subscriber('husky/cv/coords', Float64MultiArray, self.vision_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)


        # Publishers
        self.campublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    def status_callback(self, msg):
        with self.status_lock:
            for status in msg.status_list:
                if status.status == 3:  # Goal succeeded
                    self.statusin = 3
                    self.status_condition.notify_all()
                    rospy.loginfo("Goal reached successfully.")
                elif status.status == 1:  # Goal is active
                    self.statusin = 1

    def amcl_callback(self, msg):
        """Callback to update the robot's current position."""
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        rospy.loginfo(f"Updated current position: {self.current_position}")


    def vision_callback(self, msg):
        try:
            self.coords = msg.data
        except Exception as e:
            rospy.logwarn(f"Error in vision callback: {e}")


    def is_goal_reached(self, current, goal, threshold=0.5):
        """Check if the robot is within a threshold distance from the goal."""
        if current is None:
            rospy.logwarn("Current position is not available.")
            return False
        dx = current[0] - goal[0]
        dy = current[1] - goal[1]
        return (dx ** 2 + dy ** 2) ** 0.5 < threshold


    def coordinates_provider(self, coords):
        px, py, az = coords
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.pose.position.x = px
        goal_msg.pose.position.y = py
        goal_msg.pose.position.z = 0.0

        # Convert yaw to quaternion
        quaternion = quaternion_from_euler(0, 0, az)
        goal_msg.pose.orientation.x = quaternion[0]
        goal_msg.pose.orientation.y = quaternion[1]
        goal_msg.pose.orientation.z = quaternion[2]
        goal_msg.pose.orientation.w = quaternion[3]
        rospy.sleep(0.1)
        self.goal_publisher.publish(goal_msg)
        rospy.loginfo(f"Published goal: x={px}, y={py}, az={az}")

    def vision_moment(self):
        if not self.coords:
            rospy.logwarn("No coordinates received for vision-based control.")
            return

        linearx, angularz = self.coords

        # Scale velocities for finer control
        scaled_linearx = max(min(linearx * 0.5, 0.2), -0.2)  # Cap at ±0.2 m/s
        scaled_angularz = max(min(angularz * 0.5, 0.3), -0.3)  # Cap at ±0.3 rad/s

        # Publish velocity commands
        twist_msg = Twist()
        twist_msg.linear.x = scaled_linearx
        twist_msg.angular.z = scaled_angularz
        rospy.sleep(0.1)
        self.campublisher.publish(twist_msg)
        rospy.loginfo(f"Vision-based command: linear_x={linearx}, angular_z={angularz}")

    def image_callback(self):
        coordinates = [
            (3.896687984466553, -4.995651721954346, 0), #1-1
            (14.466163635253906, -5.428245544433594, 0), #1-2
            (24.652462005615234, -6.138862133026123, 0), #1-3
            (24.306838989257812, -4.293079853057861, 1), #O2-1
            (14.902565002441406, -3.401494951248169, 1), #O2-2
            (4.805065155029297, -2.686936378479004, 1), #O2-3
            (2.980816841125488, -0.8821277618408203, 0), #3-1
            (14.99194622039795, -1.6300654411315918, 0), #3-2
            (24.91880226135254, -2.3465325832366943, 0), #3-3
            (25.979534149169922, -0.6578900814056396, 1), #O4-1
            (15.133851051330566, 0.1372600793838501, 1), #O4-2
            (3.352293434143066, 0.7914607524871826, 1), #O4-3
            (2.880220413208008, 2.703320264816284, 0), #5-1
            (14.944404602050781, 1.9542776346206665, 0), #5-2
            (26.177703857421875, 1.0990581512451172, 0), #5-3
            (26.09296417236328, 2.998377799987793, 1), #O6-1
            (15.026899337768555, 3.6981594562530518, 1), #O6-2
            (3.225094318389893, 4.49996280670166, 1), #O6-3
            (2.925881385803223, 6.356810569763184, 0), #7-1
            (14.955315589904785, 5.4817705154418945, 0), #7-2
            (27.041698455810547, 4.7381391525268555, 0), #7-3
            (27.12934112548828, 6.374787330627441, 1), #O8-1
            (15.979625701904297, 7.276393890380859, 1), #O8-2
            (3.353730201721191, 8.022231565246582, 1), #O8-3
            (2.630086898803711, 10.017939567565918, 0), #9-1
            (16.047760009765625, 8.961160659790039, 0), #9-2
            (26.920825958251953, 8.297236442565918, 0), #9-3
            (25.45778465270996, 10.494369506835938, 1), #010-1
            (16.48235511779785, 11.090571403503418, 1), #010-2
            (3.761198997497559, 12.133644104003906, 1), #010-3
        ]


        for coords in coordinates:
            rospy.loginfo("Publishing new goal...")
            self.coordinates_provider(coords)

            with self.status_lock:
                while not self.is_goal_reached(self.current_position, coords, threshold=0.5):
                    self.status_condition.wait(timeout=1.0)

                    if self.statusin == 3:
                        rospy.loginfo("Goal reached. Moving to next waypoint...")
                        break
                    elif self.statusin == 1:
                        self.vision_moment()

            if self.current_position is None:
                rospy.logwarn("Skipping waypoint due to missing current position.")
                continue

            rospy.loginfo("All waypoints completed.")

    def run(self):
        rospy.loginfo("Starting navigation process.")
        self.image_callback()


if __name__ == '__main__':
    rospy.init_node('combine_cam')
    process = Process()
    process.run()
    rospy.spin()