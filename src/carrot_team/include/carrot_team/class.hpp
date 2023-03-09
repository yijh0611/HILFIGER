#ifndef CARROT_TEAM_CLASS
#define CARROT_TEAM_CLASS

#include <vector>
#include <cmath>

#include "ros/ros.h"
#include "carrot_team/poi.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float64MultiArray.h"

#include "carrot_team/orientation.hpp"

class Target_POI
{
    private:
    ros::Subscriber poi_sub_;
    carrot_team::poi point_of_interests_;

    int target_idx_;
    float target_yaw_;

    public:
    Target_POI(ros::NodeHandle *nh);

    void poi_sub_callback(const carrot_team::poi::ConstPtr &msg);

    void calculate_range(float *temp_poi, float *current_pos, float *range, float *yaw);

    void find_min_range(float *current_pos);

    void get_target_poi(float *target_poi_yaw);

};


namespace AIMS {
    class Vehicle
    {
        private:
        float current_position_[3];
        ros::Subscriber pos_sub_;
        ros::Publisher zyaw_pub_;

        public:
        Vehicle(ros::NodeHandle *nh);

        void pos_sub_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);

        void get_current_pos(float *current_position);

        void set_zoffset_yaw(float *target_poi_yaw);
    };
}


class Depth
{
    private:
    uint height_;
    uint width_;
    uint depth_size_;
    ros::Subscriber depth_sub_;
    double depth_array_[640*480];
    
    public:  
    Depth(ros::NodeHandle *nh);

    void depth_sub_callback(const std_msgs::Float64MultiArray::ConstPtr &msg);
};
#endif