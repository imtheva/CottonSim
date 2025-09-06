#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/NavSatFix.h>
#include <robot_localization/navsat_conversions.h>

// Define the action client for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Global variables
std::vector<double> husky_cv_coords(3, 0.0); // Holds CV-based coordinates (x, y, yaw)
bool use_cv_navigation = false; // Flag to check if CV navigation is active
geometry_msgs::PointStamped gps_point; // Holds GPS-based coordinates
ros::Publisher cmd_vel_pub; // Publisher to send velocity commands

// Function to convert GPS coordinates to UTM
geometry_msgs::PointStamped latLongtoUTM(double latitude, double longitude) {
    geometry_msgs::PointStamped utm_point;
    double utm_x, utm_y;
    std::string utm_zone;

    RobotLocalization::NavsatConversions::LLtoUTM(latitude, longitude, utm_y, utm_x, utm_zone);
    utm_point.point.x = utm_x;
    utm_point.point.y = utm_y;
    utm_point.header.frame_id = "utm"; // Set frame to UTM
    utm_point.header.stamp = ros::Time::now();
    return utm_point;
}

// Function to convert UTM to map points
geometry_msgs::PointStamped UTMtoMapPoint(const geometry_msgs::PointStamped& utm_point, tf2_ros::Buffer& tfBuffer) {
    geometry_msgs::PointStamped map_point;
    map_point.header = utm_point.header;
    try {
        geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("map", "utm", ros::Time(0), ros::Duration(1.0));
        tf2::doTransform(utm_point, map_point, transformStamped);
    } catch (tf2::TransformException& ex) {
        ROS_WARN("%s", ex.what());
    }
    return map_point;
}

// Function to build a move_base goal
move_base_msgs::MoveBaseGoal buildGoal(const geometry_msgs::PointStamped& point, const geometry_msgs::PointStamped& orientation, bool use_orientation) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = point.point.x;
    goal.target_pose.pose.position.y = point.point.y;
    goal.target_pose.pose.orientation.w = 1.0; // Default orientation

    if (use_orientation) {
        goal.target_pose.pose.orientation.z = sin(orientation.point.y / 2);
        goal.target_pose.pose.orientation.w = cos(orientation.point.y / 2);
    }
    return goal;
}

// Callback for GPS coordinates
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    double latitude = msg->latitude;
    double longitude = msg->longitude;

    gps_point = latLongtoUTM(latitude, longitude); // Convert to UTM
    use_cv_navigation = false; // Switch to GPS navigation
    ROS_INFO("Received GPS coordinates: lat=%.6f, lon=%.6f", latitude, longitude);
}

// Callback for CV-based coordinates (for controlling Husky's movement between GPS waypoints)
void huskyCvCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() >= 3) {
        husky_cv_coords[0] = msg->data[0]; // x-coordinate
        husky_cv_coords[1] = msg->data[1]; // y-coordinate
        husky_cv_coords[2] = msg->data[2]; // yaw angle
        use_cv_navigation = true;
        ROS_INFO("Received CV coordinates: x=%.2f, y=%.2f, yaw=%.2f", husky_cv_coords[0], husky_cv_coords[1], husky_cv_coords[2]);
    }
}

// Function to send velocity commands directly (for CV adjustments)
void sendVelocityCommands(double linear_x, double angular_z) {
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = linear_x;
    twist_msg.angular.z = angular_z;
    cmd_vel_pub.publish(twist_msg);
}

// Function to adjust the Husky's movement using CV data (to correct trajectory)
void adjustMovementWithCV() {
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;

    // Assuming CV data provides adjustments to movement:
    // Control linear speed (forward/backward) and angular velocity (turning)
    linear_velocity = 0.5; // Set a default speed
    angular_velocity = 0.5 * (husky_cv_coords[2]); // Yaw control based on CV

    sendVelocityCommands(linear_velocity, angular_velocity);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_cv_navigation");
    ros::NodeHandle n;

    // Publishers and subscribers
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber gpsSub = n.subscribe("/gps/fix", 10, gpsCallback);  // GPS topic
    ros::Subscriber huskyCvSub = n.subscribe("/husky/cv/coords", 10, huskyCvCallback); // CV coordinates topic

    // MoveBase client
    MoveBaseClient ac("move_base", true);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Wait for move_base to start
    ROS_INFO("Waiting for move_base action server...");
    ac.waitForServer();
    ROS_INFO("Connected to move_base server.");

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        // If CV coordinates are available, adjust movement
        if (use_cv_navigation) {
            ROS_INFO("Adjusting movement based on CV data...");

            adjustMovementWithCV(); // Apply CV data to adjust Husky's movement
        } else if (!gps_point.header.stamp.isZero()) {
            ROS_INFO("Moving towards GPS waypoint...");

            // Convert GPS to map frame
            geometry_msgs::PointStamped map_point_gps = UTMtoMapPoint(gps_point, tfBuffer);

            move_base_msgs::MoveBaseGoal gps_goal = buildGoal(map_point_gps, map_point_gps, true);
            ac.sendGoal(gps_goal);

            if (ac.waitForResult(ros::Duration(10.0))) {
                if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("GPS Goal reached!");
                } else {
                    ROS_WARN("Failed to reach GPS Goal!");
                }
            }
        } else {
            ROS_INFO("Waiting for GPS or CV coordinates...");
        }

        rate.sleep();
    }

    return 0;
}
