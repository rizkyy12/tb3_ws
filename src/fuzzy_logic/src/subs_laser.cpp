#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class FuzzyLogic{
    private:
        ros::Subscriber laser_subs;
        ros::Subscriber joy_feeder;
        // ros::Publisher shooter_pub;
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

    public:
        sensor_msgs::LaserScan isValue;

        FuzzyLogic(ros::NodeHandle *nh){
            // shooter_pub = nh->advertise<std_msgs::Int32>("/shooter", 10);
            // feeder_pub = nh->advertise<std_msgs::Int32>("/feeder", 10);
            laser_subs = nh->subscribe("/scan", 10, &FuzzyLogic::laser_Cb, this);

        }

        void laser_Cb(const sensor_msgs::LaserScan::ConstPtr& scan){
            std::vector<float> ranges = scan->ranges;

            // Get the number of ranges in the scan
            size_t numRanges = ranges.size();

            // Iterate over the ranges and check for obstacles
            for (size_t i = 0; i < numRanges; ++i) {
                float range = ranges[i];

                // Check if the range is below the minimum distance threshold
                if (range < 0.5) {
                    // Obstacle detected
                    ROS_INFO("Obstacle detected at range %.2f meters, index %zu", range, i);
                    // Perform further actions or control logic
                }
            }
        }

    void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg){

        float angle_min = msg->angle_min;
        float angle_max = msg->angle_max;
        float angle_increment = msg->angle_increment;

        std::vector<float> ranges = msg->ranges;

        // float desired_angle_min = -0.5;  // Minimum angle in radians
        // float desired_angle_max =  0.5;   // Maximum angle in radians

        // int index_min = (desired_angle_min - angle_min) / angle_increment;
        // int index_max = (desired_angle_max - angle_min) / angle_increment;
        
        // float front_range = 0;
        // for (int i = index_min; i <= index_max; ++i){
        //     front_range = ranges[i];
        //     if (front_range < 1)  // Set your desired threshold distance for obstacle detection
        //     {
        //         // ROS_INFO("Obstacle detected at angle %.2f with range %.2f meters", angle_min + i * angle_increment, range);
        //         ROS_INFO("front range: %f ", front_range);
        //     }    
        // }

        size_t numRanges = ranges.size();
        size_t startIdx = numRanges / 3;
        size_t endIdx = 2 * numRanges / 3;

        for (size_t i = startIdx; i < endIdx; ++i) {
        float range = ranges[i];

        // Check if the range is below the minimum distance threshold
            if (range < 0.5) {
                // Obstacle detected in the front
                ROS_INFO("Obstacle detected in the front at range %.2f meters, index %zu", range, i);
                // Perform further actions or control logic
            }
        }

        float left_range = msg->ranges[msg->ranges.size() * 1/4];
        // float front_range = msg->ranges[msg->ranges.size() / 2];
        float right_range = msg->ranges[msg->ranges.size() * 3/4];
        
        
        // float obstacle_left = fuzzyObstacle(left_range);
        // float obstacle_right = fuzzyObstacle(right_range);
        // float obstacle_front = fuzzyObstacle(front_range);

        // ROS_INFO("obstacle left: %f ", obstacle_left);
        // ROS_INFO("obstacle right: %f ", obstacle_right);
        // ROS_INFO("obstacle front: %f ", obstacle_front);    
            
    }

    // void laserScanMsgCallbac(const sensor_msgs::LaserScan::ConstPtr& msg){
    //     uint16_t scan_angle[3] = {0, 30, 330};
    //     for (int num = 0; num < 3; num++){
    //         if (std::isinf(msg->ranges.at(scan_angle[num])))
    //         {
    //         scan_data_[num] = msg->range_max;
    //         }
    //         else
    //         {
    //         scan_data_[num] = msg->ranges.at(scan_angle[num]);
    //         }
    //     }
    // }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

    float angle_min = msg->angle_min;
    float angle_max = msg->angle_max;
    float angle_increment = msg->angle_increment;

    std::vector<float> ranges = msg->ranges;

    // Define the desired angle range for obstacle detection
    float desired_angle_min = -0.5;  // Minimum angle in radians
    float desired_angle_max =  0.5;   // Maximum angle in radians

    // Calculate the corresponding array indices for the desired angle range
    int index_min = (desired_angle_min - angle_min) / angle_increment;
    int index_max = (desired_angle_max - angle_min) / angle_increment;

    // Iterate over the range values within the desired angle range
        for (int i = index_min; i <= index_max; ++i){
            float range = ranges[i];
            if (range < 1)  // Set your desired threshold distance for obstacle detection
            {
                ROS_INFO("Obstacle detected at angle %.2f with range %.2f meters", angle_min + i * angle_increment, range);
            }            
        }
    }

};




int main(int argc, char **argv)
{
    // ROS_INFO("init Node");
    ros::init(argc, argv, "fuzzy_logic");
    ros::NodeHandle nh;
    FuzzyLogic Fuzzy_Logic = FuzzyLogic(&nh);
    ros::spin();
}