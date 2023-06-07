#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

// Global variables
ros::Publisher cmdVelPub;

// Fuzzy logic parameters
double fuzzyInput;
double fuzzyOutputLinear;
double fuzzyOutputAngular;

// Fuzzy logic control parameters
const double MIN_DIST = 0.8;       // Minimum distance to consider an obstacle
const double MAX_ANGULAR = 0.5;    // Maximum angular velocity
const double MAX_LINEAR = 0.3;     // Maximum linear velocity

// Callback function for the laser scan subscriber
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::vector<float> ranges = msg->ranges;
    size_t numRanges = ranges.size();
    float front_range;
            // Iterate over the ranges and check for obstacles
    for (size_t i = 0; i < numRanges; ++i) {
        front_range = ranges[i];

                // Check if the range is below the minimum distance threshold
        if (front_range < 1.2) {
                    // Obstacle detected
            ROS_INFO("front %.2f meters", front_range);
        }
    }

    // Process the laser scan data
    // int numRanges = msg->ranges.size();
    // double minDist = std::numeric_limits<double>::max();

    // // Find the minimum distance to an obstacle
    // for (int i = 0; i < numRanges; ++i)
    // {
    //     double dist = msg->ranges[i];
    //     if (dist < minDist)
    //         minDist = dist;
    // }

    // Calculate the fuzzy input based on the minimum distance
    fuzzyInput = front_range;

    // Fuzzy logic control
    if (fuzzyInput >= MIN_DIST)
    {
        // If no obstacle, go straight
        fuzzyOutputLinear = MAX_LINEAR;
        fuzzyOutputAngular = 0.0;
    }
    else
    {
        // If obstacle detected, turn
        fuzzyOutputLinear = 0.0;
        fuzzyOutputAngular = MAX_ANGULAR;
    }

    // Publish the output as a velocity command
    geometry_msgs::Twist cmdVelMsg;
    cmdVelMsg.linear.x = fuzzyOutputLinear;
    cmdVelMsg.angular.z = fuzzyOutputAngular;
    cmdVelPub.publish(cmdVelMsg);
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle nh;

    // Create the publisher and subscriber
    cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber laserScanSub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laserScanCallback);

    // Start the main ROS loop
    ros::spin();

    return 0;
}