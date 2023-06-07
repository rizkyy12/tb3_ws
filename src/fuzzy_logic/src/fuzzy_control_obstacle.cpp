#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class FuzzyLogic{
    private:
        ros::Subscriber laser_subs;
        ros::Publisher vel_pub;
        // ros::Publisher feeder_pub;

    public:
        const double MIN_DISTANCE_FRONT = 0.9;
        const double MIN_DISTANCE_LEFT = 0.8;
        const double MIN_DISTANCE_RIGHT = 0.8;
        const double MAX_ANGULAR_VEL = 0.5;
        const double MIN_ANGULAR_VEL = 0.15;
        const double MAX_LINEAR_VEL = 0.2;
        const double MIN_LINEAR_VEL = 0.1;

        double fuzzyOutputLinear;
        double fuzzyOutputAngular;
        double fuzzyInput;

        sensor_msgs::LaserScan isValue;

        FuzzyLogic(ros::NodeHandle *nh){
            // shooter_pub = nh->advertise<std_msgs::Int32>("/shooter", 10);
            // feeder_pub = nh->advertise<std_msgs::Int32>("/feeder", 10);
            laser_subs = nh->subscribe("/scan", 10, &FuzzyLogic::laser_Cb, this);
            vel_pub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        }

        void laser_Cb(const sensor_msgs::LaserScan::ConstPtr& msg){

            std::vector<float> ranges = msg->ranges;
            geometry_msgs::Twist vel;

            vel.linear.x = 0.0;
            vel.angular.z = 0.0;

            // Get the number of ranges in the scan
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

            float left_range = msg->ranges[msg->ranges.size() * 1/4];
            float right_range = msg->ranges[msg->ranges.size() * 3/4];
            ROS_INFO("left %.2f meters", left_range);
            ROS_INFO("right %.2f meters", right_range);
            
            //Fuzzy Rule
            // Fuzzy logic control rules for angular velocity
            // Rule 1: Jika ada halangan di kiri, belok kanan
            // Rule 2: Jika ada halangan di depan, belok kanan
            // Rule 3: Jika ada halangan di kanan, belok kiri
            // Rule 4: Jika tidak ada halangan, jalanS aja lurus

            // Fuzzy logic control rules for linear velocity
            // Rule 5: jika terdapat halangan di sisi manapun, mengurangi kecepatan linear
            // Rule 6: jika tidak ada halangan, mempertahankan kecepatan linear
            //  ======================================== Implementasi fuzzy logic dari matlab ================================
            if (front_range < MIN_DISTANCE_FRONT){
                vel.angular.z = MAX_ANGULAR_VEL;
                vel.linear.x = 0.0;
            }

            else if (front_range > MIN_DISTANCE_FRONT && right_range < MIN_DISTANCE_RIGHT){
                vel.angular.z = MIN_ANGULAR_VEL;
                vel.linear.x = MAX_LINEAR_VEL;
            }

            else if (front_range < MIN_DISTANCE_FRONT && left_range < MIN_DISTANCE_LEFT){
                vel.angular.z = -MAX_ANGULAR_VEL;
                vel.linear.x = 0.0;
            } 

            else if (right_range < MIN_DISTANCE_RIGHT){
                vel.angular.z = MIN_ANGULAR_VEL;
                vel.linear.x = MIN_LINEAR_VEL;
            }

            else if (left_range < MIN_DISTANCE_LEFT){
                vel.angular.z = -MAX_ANGULAR_VEL;
                vel.linear.x = 0.0;
            }
            else if (left_range < MIN_DISTANCE_LEFT){
                vel.angular.z = -MAX_ANGULAR_VEL;
                vel.linear.x = 0.0;
            }

            else if (right_range > MIN_DISTANCE_RIGHT || front_range > MIN_DISTANCE_FRONT || left_range > MIN_DISTANCE_LEFT){
                vel.angular.z = 0.0;
                vel.linear.x = MAX_LINEAR_VEL;
            }

            else if (front_range > MIN_DISTANCE_FRONT && right_range < MIN_DISTANCE_RIGHT && left_range < MIN_DISTANCE_LEFT){
                vel.angular.z = 0.0;
                vel.linear.x = MAX_LINEAR_VEL;
            }

            else if (front_range < MIN_DISTANCE_FRONT && right_range < MIN_DISTANCE_RIGHT && left_range < MIN_DISTANCE_LEFT){
                vel.angular.z = -MAX_ANGULAR_VEL;
                vel.linear.x = 0.0;
            }

            else if (front_range > MIN_DISTANCE_FRONT && right_range < MIN_DISTANCE_RIGHT && left_range > MIN_DISTANCE_LEFT){
                vel.angular.z = 0.0;
                vel.linear.x = MAX_LINEAR_VEL;
            }

            else if (front_range > MIN_DISTANCE_FRONT && right_range > MIN_DISTANCE_RIGHT && left_range < MIN_DISTANCE_LEFT){
                vel.angular.z = 0.0;
                vel.linear.x = MAX_LINEAR_VEL;
            }

            else if (front_range > MIN_DISTANCE_FRONT && right_range < MIN_DISTANCE_RIGHT && left_range > MIN_DISTANCE_LEFT){
                vel.angular.z = 0.0;
                vel.linear.x = MAX_LINEAR_VEL;
            }

            else {
                vel.angular.z = 0.0;
                vel.angular.x = 0.0;
            }

            // else if (front_range < MIN_DISTANCE_FRONT && right_range < MIN_DISTANCE_RIGHT && left_range < MIN_DISTANCE_LEFT){
            //     vel.angular.z = -MAX_ANGULAR_VEL;
            //     vel.linear.x = 0.0;
            // }

            // else if (front_range > MIN_DISTANCE_FRONT && right_range < MIN_DISTANCE_RIGHT && left_range > MIN_DISTANCE_LEFT){
            //     vel.angular.z = 0.0;
            //     vel.linear.x = MAX_LINEAR_VEL;
            // }

            // else if (front_range > MIN_DISTANCE_FRONT && right_range > MIN_DISTANCE_RIGHT && left_range < MIN_DISTANCE_LEFT){
            //     vel.angular.z = 0.0;
            //     vel.linear.x = MAX_LINEAR_VEL;
            // }

            // else if (front_range > MIN_DISTANCE_FRONT && right_range < MIN_DISTANCE_RIGHT && left_range > MIN_DISTANCE_LEFT){
            //     vel.angular.z = 0.0;
            //     vel.linear.x = MAX_LINEAR_VEL;
            // }

            // else {
            //     vel.angular.z = 0.0;
            //     vel.angular.x = 0.0;
            // }

            //  ==================================== END Implementasi fuzzy logic dari matlab ================================

            // rule 1: Jika ada halangan di kiri, belok kanan
            // if (left_range < MIN_DISTANCE_LEFT){
            //     vel.angular.z = -MAX_ANGULAR_VEL;
            //     vel.linear.x = 0.0;
            // }
            // // Rule 3: Jika ada halangan di kanan, belok kiri
            // else if (right_range < MIN_DISTANCE_RIGHT){
            //     vel.angular.z = MAX_ANGULAR_VEL;
            //     vel.linear.x = 0.0;
            // }

            // else if (right_range < MIN_DISTANCE_RIGHT && left_range < MIN_DISTANCE_LEFT){
            //     vel.angular.z = 0.0;
            //     vel.linear.x = MAX_LINEAR_VEL;
            // }
            // else if (right_range < MIN_DISTANCE_RIGHT && front_range > MIN_DISTANCE_FRONT){
            //     vel.angular.z = 0.0;
            //     vel.linear.x = MAX_LINEAR_VEL;
            // }
            // // Rule 2: Jika ada halangan di depan, belok KIRI
            // else if (front_range < MIN_DISTANCE_FRONT){
            //     vel.angular.z = MAX_ANGULAR_VEL;
            //     vel.linear.x = 0.0;
            // }
            // else if (front_range > MIN_DISTANCE_FRONT && left_range < MIN_DISTANCE_LEFT){
            //     vel.angular.z = 0.0;
            //     vel.linear.x = MAX_LINEAR_VEL;
            // }
            // // Rule 4: Jika tidak ada halangan, jalan aja lurus
            // else if (right_range > MIN_DISTANCE_RIGHT || front_range > MIN_DISTANCE_FRONT || left_range > MIN_DISTANCE_LEFT){
            //     vel.angular.z = 0.0;
            //     vel.linear.x = MAX_LINEAR_VEL;
            // }

            // // Rule 5: jika terdapat halangan di sisi manapun, mengurangi kecepatan linear
            // else if (right_range < MIN_DISTANCE_RIGHT && left_range < MIN_DISTANCE_LEFT && front_range < MIN_DISTANCE_FRONT){
            //     vel.angular.z = 0.0;
            //     vel.linear.x = 0.0;
            // }
            // // Rule 6: jika tidak ada halangan, mempertahankan kecepatan linear    
            // else if (right_range > MIN_DISTANCE_RIGHT && left_range > MIN_DISTANCE_LEFT && front_range > MIN_DISTANCE_FRONT){
            //     vel.angular.z = 0.0;
            //     vel.linear.x = MAX_LINEAR_VEL;
            // }

            // else {
            //     vel.angular.z = 0.0;
            //     vel.linear.x = 0.0;
            // }

            // else if(front_range < 0.7 && right_range < 0.6 ){
            //     vel.linear.x = 0.3;
            //     vel.angular.z = 0.0;
            // }
            // else if (front_range < 0.7 && right_range < 0.6){
            //     vel.linear.x = 0.0;
            //     vel.angular.z = 0.6;
            // }
            // else if (front_range < 0.7 && left_range < 0.6){
            //     vel.linear.x = 0.0;
            //     vel.angular.z = -0.6;
            // }
            // else if (front_range < 0.7 && left_range < 0.6 && right_range < 0.6){
            //     vel.linear.x = 0.0;
            //     vel.angular.z = 3.0;
            // }
            // else if(front_range > 1){
            //     vel.linear.x = 0.5;
            //     vel.angular.z = 0.0;
            // }
            // else if(front_range < 1 || left_range < 1 || right_range < 1){
            //     vel.linear.x = 0.1;
            // }
            // else if(left_range < 1 && right_range < 1){
            //     vel.linear.x = 0.1;
            // }
            // else {
            //     vel.angular.z = 0.0;
            //     vel.angular.x = 0.0;
            // }

            // angular
            // jika ada halangan di kiri
            // if (front_range > 0.7){
            //     vel.angular.z = 0.0;
            //     vel.linear.x = 0.1;
            // }
            // if (front_range < 0.7){
            //     vel.angular.z = 0.5;
            //     vel.linear.x = 0.0;
            // }
            // // jika jauh dari halangan
            // else if (front_range < 0.7){
            //     vel.angular.z = 0.5;
            //     vel.linear.x = 0.0;
            // }
            // else if (right_range < 0.7){
            //     vel.angular.z = 0.5;
            //     vel.linear.x = 0.0;
            // }
            // else if (front_range < 0.6 && right_range > 0.7 && left_range > 0.7){
            //     vel.angular.z = 0.5;
            //     vel.linear.x = 0;
            // }
            // else if (right_range < 0.7 && left_range > 0.7 && front_range < 0.8){
            //     vel.angular.z = 0.5;
            //     vel.linear.x = 0.0;
            // }
            // else if (right_range < 0.7 && front_range < 0.8){
            //     vel.angular.z = 0.5;
            //     vel.linear.x = 0.0;
            // }

            // else if (left_range < 0.8){
            //     vel.angular.z = -0.5;
            //     vel.linear.x = 0.0;
            // }
            // //jika ada halangan di depan
            // else if (front_range < 0.8){
            //     vel.angular.z = 1;
            //     vel.linear.x = 0.0;
            // }

            // else if (front_range < 0.8 && right_range < 0.7){
            //     vel.angular.z = 0.3;
            //     vel.linear.x = 0.0;
            // }
            // else if (front_range < 0.8 && left_range < 0.7){
            //     vel.angular.z = -0.3;
            //     vel.linear.x = 0.0;
            // }
            // else if (front_range < 0.8 && left_range < 0.7 && right_range < 0.7){
            //     vel.angular.z = -0.3;
            //     vel.linear.x = 0.0;
            // }
            // jika ada halangan di depan dan kanan
            // else if (front_range < 0.7 && right_range < 0.6){
            //     vel.angular.z = 0.2;
            //     vel.linear.x = 0.0;
            // }
            // // jika ada halangan di depan dan kiri
            // else if (front_range < 0.7 && left_range < 0.6){
            //     vel.angular.z = -0.2;
            //     vel.linear.x = 0.0;
            // }
            // // jika ada halangan di kiri, depan dan kanan
            // else if (left_range < 0.6 && right_range < 0.6 && front_range < 0.7){
            //     vel.angular.z = 0.2;
            //     vel.linear.x = 0.0;
            // }
            // linear

            // else {
            //     vel.angular.z = 0.0;
            //     vel.angular.x = 0.0;
            // }

            ROS_INFO("Lin vel: %f ", vel.linear.x);
            ROS_INFO("Angular vel: %f ", vel.angular.z);
            vel_pub.publish(vel);
        }
        
};

int main(int argc, char **argv)
{
    // ROS_INFO("init Node");
    ros::init(argc, argv, "fuzzy_obstacle_avoidance");
    ros::NodeHandle nh;
    FuzzyLogic Fuzzy_Logic = FuzzyLogic(&nh);
    ros::spin();
}