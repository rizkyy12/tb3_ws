#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class ObstacleAvoidance{
    private:
        ros::Subscriber laser_subs;
        ros::Publisher vel_pub;
        // ros::Publisher feeder_pub;

    float fuzzyObstacle(float range){
        double max_range = 3.5;
        double jarak_batas = 0.2;

        if (range < jarak_batas){
            return 1;
        }
        else if(range >= max_range){
            return 0;
        }
        else{
            return 1 - (range - jarak_batas) / (max_range / jarak_batas);
        }

        return 0;
    }

    float fuzzyRule(float obstacle_left, float obstacle_front, float obstacle_right){
        // Fuzzy logic control rules for angular velocity
        // Rule 1: Jika ada halangan di kiri, belok kanan
        // Rule 2: Jika ada halangan di depan, belok kanan
        // Rule 3: Jika ada halangan di kanan, belok kiri
        // Rule 4: Jika tidak ada halangan, jalan aja lurus

        // Fuzzy logic control rules for linear velocity
        // Rule 5: jika terdapat halangan di sisi manapun, mengurangi kecepatan linear
        // Rule 6: jika tidak ada halangan, mempertahankan kecepatan linear

        float angular_vel = 0.0;
        float linear_vel = 0.0;

        // angular vel
        if (obstacle_left < 0.35){
            linear_vel = 0.0;
            angular_vel = 1.0;
        }
        else if (obstacle_front < 0.35){
            angular_vel = 1.0;
        }
        else if (obstacle_right < 0.21){
            angular_vel = -1.0;
        }
        else {
            angular_vel = 0.0;
        }

        // linear vel 
        if (obstacle_left > 0.21 || obstacle_front > 0.25 || obstacle_right > 0.21){
            linear_vel = 0.1;
        }
        else{
            linear_vel = 0.0;
        }

        return angular_vel, linear_vel;
    }

    public:
        
        ObstacleAvoidance(ros::NodeHandle *nh){
            laser_subs = nh->subscribe("/scan", 10, &ObstacleAvoidance::scanCb, this);
            vel_pub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        }

    void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg){
        // float min_range = *std::min_element(msg->ranges.begin(), msg->ranges.end());
        // ROS_INFO("Minimum range: %f ", min_range);
        float left_range = msg->ranges[msg->ranges.size() * 1/4];
        float front_range = msg->ranges[msg->ranges.size() / 2];
        float right_range = msg->ranges[msg->ranges.size() * 3/4];

        // float obstacle_left = fuzzyObstacle(left_range);
        // float obstacle_right = fuzzyObstacle(right_range);
        // float obstacle_front = fuzzyObstacle(front_range);

        //rules
        float angular_vel = fuzzyRule(left_range, front_range, right_range);
        float linear_vel = fuzzyRule(left_range, front_range, right_range);

        ROS_INFO("Lin vel: %f ", linear_vel);
        
        geometry_msgs::Twist vel;
        vel.linear.x = 0.1;
        vel.angular.z = angular_vel;
        vel_pub.publish(vel);
    }

};



int main(int argc, char **argv)
{
    // ROS_INFO("init Node");
    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle nh;
    ObstacleAvoidance Obstacle_Avoidance = ObstacleAvoidance(&nh);
    ros::spin();
    return 0;
}