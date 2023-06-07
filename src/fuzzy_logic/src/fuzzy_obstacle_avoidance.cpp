#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

// Fuzzy logic membership functions
double distanceMembership(double distance) {
    // Define membership functions for distance
    // You can customize these functions based on your specific requirements
    if (distance <= 1.0) {
        return 1.0;
    } else if (distance > 1.0 && distance <= 3.0) {
        return (3.0 - distance) / 2.0;
    } else {
        return 0.0;
    }
}

double angleMembership(double angle) {
    // Define membership functions for angle
    // You can customize these functions based on your specific requirements
    if (angle <= -45.0 || angle >= 45.0) {
        return 1.0;
    } else if (angle > -45.0 && angle < 0.0) {
        return (angle + 45.0) / 45.0;
    } else if (angle > 0.0 && angle < 45.0) {
        return (45.0 - angle) / 45.0;
    } else {
        return 0.0;
    }
}

// Fuzzy logic defuzzification
double defuzzify(double fuzzyOutput) {
    // Perform defuzzification, e.g., weighted average
    // You can customize the defuzzification method based on your specific requirements
    double crispOutput = fuzzyOutput; // Scale the fuzzy output to the desired range
    return crispOutput;
}

ros::Publisher cmdVelPub;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Callback function to process laser scan data

    // Extract necessary information from the laser scan message
    std::vector<float> ranges = msg->ranges;
    int numReadings = ranges.size();
    float angleIncrement = msg->angle_increment;

    // Calculate the angle to the closest obstacle
    int closestIndex = 0;
    float closestRange = std::numeric_limits<float>::max();
    for (int i = 0; i < numReadings; ++i) {
        float range = ranges[i];

        if (range < closestRange) {
            closestRange = range;
            closestIndex = i;
        }
    }
    float angle = (closestIndex * angleIncrement) - (numReadings / 2) * angleIncrement;

    // Perform fuzzy logic control
    double fuzzyDistance = distanceMembership(closestRange);
    double fuzzyAngle = angleMembership(angle);

    // Fuzzy rule evaluation
    double fuzzyOutput = std::max(fuzzyDistance, fuzzyAngle);

    // Defuzzification
    double crispOutput = defuzzify(fuzzyOutput);

    // Create a twist message to control the robot's movement
    geometry_msgs::Twist twistMsg;

    // Set the linear and angular velocities based on fuzzy logic output
    if (crispOutput >= 75.0) {
        // High priority action, e.g., stop and turn
        twistMsg.linear.x = 0.0;
        twistMsg.angular.z = 1.0;  // Rotate the robot to avoid the obstacle
    } else if (crispOutput >= 25.0) {
        // Medium priority action, e.g., slow down
        twistMsg.linear.x = 0.2;  // Move the robot forward at 0.2 m/s
        twistMsg.angular.z = 0.0;
    } else {
        // Low priority action, e.g., continue moving
        twistMsg.linear.x = 0.5;  // Move the robot forward at 0.5 m/s
        twistMsg.angular.z = 0.0;
    }
    ROS_INFO("crispOUtput: %f", crispOutput);
    ROS_INFO("angle: %f", angle);

    // Publish the twist message to control the robot's movement
    // cmdVelPub.publish(twistMsg);
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "fuzzy_obstacle_avoidance_node");
    ros::NodeHandle nh;

    // Create a publisher for the cmd_vel topic to control the robot's movement
    cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // Create a subscriber for the laser scan topic to receive sensor data
    ros::Subscriber laserSub = nh.subscribe<sensor_msgs::LaserScan>("scan", 10, laserCallback);

    // Spin the node and process callbacks
    ros::spin();

    return 0;
}