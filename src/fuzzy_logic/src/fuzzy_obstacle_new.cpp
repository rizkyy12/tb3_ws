#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

// Global variables
ros::Publisher cmdVelPub;

// Fuzzy logic parameters
double fuzzyInputDistance;
double fuzzyInputAngle;
double fuzzyInputSpeed;
double fuzzyOutputAngular;

// Fuzzy logic control parameters
const double CLOSE_DIST_THRESHOLD = 0.5;
const double ANGLE_THRESHOLD = 0.5;
const double SLOW_SPEED_THRESHOLD = 0.2;
const double MAX_ANGULAR = 1.0;

// Fuzzy rule weights
const double WEIGHT_CLOSE_SMALL_SLOW = 0.8;
const double WEIGHT_CLOSE_SMALL_MEDIUM = 0.6;
const double WEIGHT_CLOSE_SMALL_FAST = 0.4;
const double WEIGHT_CLOSE_MEDIUM_SLOW = 0.6;
const double WEIGHT_CLOSE_MEDIUM_MEDIUM = 0.4;
const double WEIGHT_CLOSE_MEDIUM_FAST = 0.2;
const double WEIGHT_CLOSE_LARGE_SLOW = 0.4;
const double WEIGHT_CLOSE_LARGE_MEDIUM = 0.2;
const double WEIGHT_CLOSE_LARGE_FAST = 0.0;
const double WEIGHT_FAR_SMALL_SLOW = 0.2;
const double WEIGHT_FAR_SMALL_MEDIUM = 0.0;
const double WEIGHT_FAR_SMALL_FAST = 0.0;
const double WEIGHT_FAR_MEDIUM_SLOW = 0.0;
const double WEIGHT_FAR_MEDIUM_MEDIUM = 0.2;
const double WEIGHT_FAR_MEDIUM_FAST = 0.4;
const double WEIGHT_FAR_LARGE_SLOW = 0.0;
const double WEIGHT_FAR_LARGE_MEDIUM = 0.2;
const double WEIGHT_FAR_LARGE_FAST = 0.4;

// Callback function for the laser scan subscriber
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // Process the laser scan data
    int numRanges = msg->ranges.size();
    double minDist = std::numeric_limits<double>::max();
    double minAngle = 0.0;

    // Find the minimum distance and corresponding angle to an obstacle
    for (int i = 0; i < numRanges; ++i)
    {
        double dist = msg->ranges[i];
        if (dist < minDist)
        {
            minDist = dist;
            minAngle = msg->angle_min + i * msg->angle_increment;
        }
    }

    // Calculate the fuzzy inputs based on the minimum distance and angle
    fuzzyInputDistance = minDist;
    fuzzyInputAngle = fabs(minAngle);

    // Fuzzy logic control
    if (fuzzyInputDistance <= CLOSE_DIST_THRESHOLD)
    {
        if (fuzzyInputAngle <= ANGLE_THRESHOLD)
        {
            if (fuzzyInputSpeed <= SLOW_SPEED_THRESHOLD)
            {
                fuzzyOutputAngular = -MAX_ANGULAR * WEIGHT_CLOSE_SMALL_SLOW;
            }
            else
            {
                fuzzyOutputAngular = -MAX_ANGULAR * WEIGHT_CLOSE_SMALL_MEDIUM;
            }
        }
        else
        {
            if (fuzzyInputSpeed <= SLOW_SPEED_THRESHOLD)
            {
                fuzzyOutputAngular = -MAX_ANGULAR * WEIGHT_CLOSE_MEDIUM_SLOW;
            }
            else
            {
                fuzzyOutputAngular = -MAX_ANGULAR * WEIGHT_CLOSE_MEDIUM_MEDIUM;
            }
        }
    }
    else
    {
        if (fuzzyInputAngle <= ANGLE_THRESHOLD)
        {
            if (fuzzyInputSpeed <= SLOW_SPEED_THRESHOLD)
            {
                fuzzyOutputAngular = MAX_ANGULAR * WEIGHT_FAR_SMALL_SLOW;
            }
            else
            {
                fuzzyOutputAngular = MAX_ANGULAR * WEIGHT_FAR_SMALL_MEDIUM;
            }
        }
        else
        {
            if (fuzzyInputSpeed <= SLOW_SPEED_THRESHOLD)
            {
                fuzzyOutputAngular = MAX_ANGULAR * WEIGHT_FAR_MEDIUM_SLOW;
            }
            else
            {
                fuzzyOutputAngular = MAX_ANGULAR * WEIGHT_FAR_MEDIUM_MEDIUM;
            }
        }
    }

    // Publish the output as a velocity command
    geometry_msgs::Twist cmdVelMsg;
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