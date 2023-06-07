#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

// Fuzzy membership functions for range
float near = 0.4;       // Range values closer than 0.3 meters are considered "near"
float medium = 1;     // Range values between 0.3 and 0.6 meters are considered "medium"
float far = 2.0;        // Range values beyond 0.6 meters are considered "far"


class ObstacleAvoid{
    private:
        ros::Subscriber laser_subs;
        ros::Publisher vel_pub;

    float fuzzyObstacle(float range){
            // Define fuzzy membership functions
        float close = 0.4;     // Range values closer than 0.3 meters are considered "close"
        float moderate = 1.0;  // Range values between 0.3 and 0.6 meters are considered "moderate"
        float far = 2.0;       // Range values beyond 0.6 meters are considered "far"

        // Define fuzzy control commands
        float sharp_left = 4;     // Control command for sharp left turn
        float slight_left = 2;    // Control command for slight left turn
        float slight_right = -2;  // Control command for slight right turn
        float sharp_right = -4;   // Control command for sharp right turn
        float keep_straight = 0.0;  // Control command for moving straight

        // Fuzzy logic calculations and inference
        float control_command = 0.0;

        if (range < close){
            control_command = sharp_right;
        }
        else if (range >= close && range < moderate){
            control_command = slight_right;
        }
        else if (range >= moderate && range < far){
            control_command = keep_straight;
        }
        else{
            control_command = slight_left;
        }

        return control_command;
    }


    public:
        
        ObstacleAvoid(ros::NodeHandle *nh){

            laser_subs = nh->subscribe("/scan", 10, &ObstacleAvoid::lidarCallback, this);
            vel_pub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        }

    // Callback function for LiDAR sensor data
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        // Process the LiDAR data and obtain the distance to the nearest obstacle
        float minDistance = std::numeric_limits<float>::infinity();
        for (float range : scan->ranges) {
            if (range < minDistance) {
                minDistance = range;
                ROS_INFO("RANGE: %f", minDistance);
            }
        }

        

        // Apply the angular velocity to the TurtleBot3
        // geometry_msgs::Twist twistMsg;
        // twistMsg.angular.z = angularVel;

        // // Publish the Twist message
        // vel_pub.publish(twistMsg);
    }


    void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg){

        geometry_msgs::Twist twist;

        // std::vector<float> ranges = msg->ranges;
        // float front_range = ranges[ranges.size() / 2];


        float angle_min = msg->angle_min;
        float angle_max = msg->angle_max;
        float angle_increment = msg->angle_increment;

        std::vector<float> ranges = msg->ranges;
        float desired_angle_min = -0.5;  // Minimum angle in radians
        float desired_angle_max =  0.5;   // Maximum angle in radians

        int index_min = (desired_angle_min - angle_min) / angle_increment;
        int index_max = (desired_angle_max - angle_min) / angle_increment;
        
        float front_range = 0;


        // float left_range = msg->ranges[msg->ranges.size() * 1/4];
        // // float front_range = msg->ranges[msg->ranges.size() / 2];
        // float right_range = msg->ranges[msg->ranges.size() * 3/4];
        
        for (int i = index_min; i <= index_max; ++i){
            front_range = ranges[i];
            if (front_range < 2)  // Set your desired threshold distance for obstacle detection
            {
                ROS_INFO("front range: %f ", front_range);
            }        
        }
        
        float control_command = fuzzyObstacle(front_range);
        ROS_INFO("front range: %f ", front_range);
        ROS_INFO("control command: %f ", control_command);
        

        twist.linear.x = 0.1;
        twist.angular.z = control_command;

        // vel_pub.publish(twist); 

    }

    void laserCb(const sensor_msgs::LaserScan::ConstPtr& msg){

        

    }

};

int main(int argc, char **argv)
{
    // ROS_INFO("init Node");
    ros::init(argc, argv, "obstacle_avoid");
    ros::NodeHandle nh;
    ObstacleAvoid Obstacle_Avoid = ObstacleAvoid(&nh);
    ros::spin();
    return 0;
}