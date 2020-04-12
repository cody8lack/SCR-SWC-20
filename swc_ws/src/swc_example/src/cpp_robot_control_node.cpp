#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <tf/transform_datatypes.h>
#include "swc_msgs/Control.h"
#include "swc_msgs/Waypoints.h"
#include "swc_msgs/Gps.h"

ros::Publisher g_control_pub;

double heading = 0;
double targetHeading = 0;

int waypointIndex = 1;

double latitude;
double longitude;

swc_msgs::Waypoints waypoints;

double angleDiff(double angle1, double angle2) {
    double diff = fmod(angle2 - angle1 + M_PI, 2 * M_PI);
    if (diff < 0) {
        diff += 2 * M_PI;
    }
    return diff - M_PI;
}

void controlTimerCallback(const ros::TimerEvent& timer_event) {
    swc_msgs::Control controlMsg;
    controlMsg.speed = 100;
    // Check if the robot has hit the current waypoint (TODO: how close counts as a hit?)
    if (sqrt(pow(waypoints.response.waypoints[waypointIndex].longitude  - longitude, 2.0) + pow(waypoints.response.waypoints[waypointIndex].latitude  - latitude, 2.0)) < 0.00001) {
       if (waypointIndex < 4) {
           // Go to the next waypoint
           ++waypointIndex;
       }
    }
    targetHeading = atan2(waypoints.response.waypoints[waypointIndex].longitude  - longitude, waypoints.response.waypoints[waypointIndex].latitude - latitude);
    double diff = angleDiff(targetHeading, heading);
    //ROS_INFO("Current heading: [%f], Target: [%f], Diff: [%f]", heading, targetHeading, diff);
    //ROS_INFO("[%f]", sqrt(pow(waypoints.response.waypoints[waypointIndex].longitude  - longitude, 2.0) + pow(waypoints.response.waypoints[waypointIndex].latitude  - latitude, 2.0)));

    // Set turn angle proportional to the difference between the target heading and the current heading
    controlMsg.turn_angle = 30.0 * diff / M_PI;

    // Publish the message to /sim/control so the simulator receives it
    g_control_pub.publish(controlMsg);
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    heading = tf::getYaw(msg->orientation);
    //ROS_INFO("[%f]", heading);
}

void gpsCallback(const swc_msgs::Gps::ConstPtr& msg) {
    latitude = msg->latitude;
    longitude = msg->longitude;
}

int main(int argc, char **argv)
{
    // Initalize our node in ROS
    ros::init(argc, argv, "cpp_robot_control_node");
    ros::NodeHandle node;

    // Create a Publisher that we can use to publish messages to the /sim/control topic
    g_control_pub = node.advertise<swc_msgs::Control>(node.resolveName("/sim/control"), 1);

    ros::Subscriber imuSub = node.subscribe("/sim/imu", 1, imuCallback);
    ros::Subscriber gpsSub = node.subscribe("/sim/gps" , 1, gpsCallback);

    // Create and wait for Waypoints service
    ros::ServiceClient waypoint_service = node.serviceClient<swc_msgs::Waypoints>("/sim/waypoints");
    waypoint_service.waitForExistence();

    // Request waypoints and display them
    swc_msgs::Waypoints waypoints_msg;
    waypoint_service.call(waypoints_msg);

    waypoints = waypoints_msg;
    std::cout << "Found the following waypoints:" << std::endl;
    for (int i=0; i<waypoints_msg.response.waypoints.size(); i++) {
        std::cout << waypoints_msg.response.waypoints[i] << std::endl;
    }

    // Create a timer that calls timer_callback() with a period of 0.1 (10 Hz)
    ros::Timer control_timer = node.createTimer(ros::Duration(0.1), &controlTimerCallback, false);

    // Let ROS take control of this thread until a ROS wants to kill
    ros::spin();

    return 0;
}