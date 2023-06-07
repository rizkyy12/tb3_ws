#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

ros::Publisher cmdVelPub;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Callback function to process laser scan data

    // Extract necessary information from the laser scan message
    std::vector<float> ranges = msg->ranges;
    int numReadings = ranges.size();
    float minRange = msg->range_min;
    float maxRange = msg->range_max;
    float angleIncrement = msg->angle_increment;

    // Perform obstacle avoidance logic
    bool obstacleDetected = false;
    for (int i = 0; i < numReadings; ++i) {
        float range = ranges[i];

        // Check if an obstacle is within a certain range threshold
        if (range < 1.0) {
            obstacleDetected = true;
            break;
        }
    }

    // Create a twist message to control the robot's movement
    geometry_msgs::Twist twistMsg;

    // Set the linear and angular velocities based on obstacle detection
    if (obstacleDetected) {
        // Obstacle detected, stop and turn
        twistMsg.linear.x = 0.0;
        twistMsg.angular.z = 1.0;  // Rotate the robot to avoid the obstacle
    } else {
        // No obstacle detected, continue moving forward
        twistMsg.linear.x = 0.3;  // Move the robot forward at 0.5 m/s
        twistMsg.angular.z = 0.0;
    }

    // Publish the twist message to control the robot's movement
    // cmdVelPub.publish(twistMsg);
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "obstacle_avoidance_node");
    ros::NodeHandle nh;

    // Create a publisher for the cmd_vel topic to control the robot's movement
    cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // Create a subscriber for the laser scan topic to receive sensor data
    ros::Subscriber laserSub = nh.subscribe<sensor_msgs::LaserScan>("scan", 10, laserCallback);

    // Spin the node and process callbacks
    ros::spin();

    return 0;
}