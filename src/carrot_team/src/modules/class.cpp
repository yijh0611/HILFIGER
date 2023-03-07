#include <vector>
#include <cmath>

#include "ros/ros.h"
#include "carrot_team/poi.h"
#include "geometry_msgs/PoseStamped.h"

#include "carrot_team/orientation.hpp"
#include "carrot_team/class.hpp"

class Target_POI
{
    private:
    ros::Subscriber poi_sub_;
    carrot_team::poi point_of_interests_;

    vector<bool> passed_poi_;
    int target_idx_;
    float target_yaw_;

    public:
    Target_POI(ros::NodeHandle *nh) {
        target_idx_ = -1;
        poi_sub_ = nh->subscribe("/red/poi", 100, &Target_POI::poi_sub_callback, this);
    }
    
    void poi_sub_callback(const carrot_team::poi &msg) {
        /* Temporary poi -> permanent poi */
        int poi_len = msg->poi.size();
        point_of_interests_.poi.resize(poi_len);
        passed_poi_.resize(poi_len);

        int _i;
        for (_i = 0; _i < poi_len; ++i) {
            point_of_interests_.poi[_i].x = msg->poi[_i].x;
            point_of_interests_.poi[_i].y = msg->poi[_i].y;
            point_of_interests_.poi[_i].z = msg->poi[_i].z;
        }
    }

    void calculate_range(float *temp_poi, *current_pos, float *range, float *yaw) {
        float delta_x, delta_y, delta_z;
        delta_x = current_pos[0] - temp_poi[0];
        delta_y = current_pos[1] - temp_poi[1];
        delta_z = current_pos[2] - temp_poi[2];
        *range = (delta_x * delta_x) + (delta_y * delta_y) + (delta_z * delta_z);
        *yaw = atan(delta_x / delta_y);
    }

    void find_min_range(float *current_pos) {
        /* find minimum range among poi */
        float range;
        float min_range = 10e9f;
 
        int len = point_of_interests_->poi.size();
        float temp_poi[3] = {0, 0, 0};
        int _i;
        for (i=0; i<len; ++i) {
            temp_poi[0] = point_of_interests_->poi[_i].x;
            temp_poi[1] = point_of_interests_->poi[_i].y;
            temp_poi[2] = point_of_interests_->poi[_i].z;
            calculate_range(temp_poi, current_pos, &range, &target_yaw_);

            if (min_range >= range) {
                min_range = range;
                target_idx_ = i;
            }
        }
        passed_poi[target_idx_] = true;
        ROS_INFO("target index of poi is [%d]", target_idx);
        ROS_INFO("target yaw of poi is [%f]", target_yaw_);
    }

    void get_target_poi(float *target_poi_yaw) {
        /* send the target poi to outside */
        if (target_idx_ >= 0) {
            target_poi_yaw[0] = point_of_interests_->poi[target_idx_].x;
            target_poi_yaw[1] = point_of_interests_->poi[target_idx_].y;
            target_poi_yaw[2] = point_of_interests_->poi[target_idx_].z;
            target_poi_yaw[3] = target_yaw_;
        }
        else {
            ROS_INFO("target idx is not defined yet");
        }
    }
}


namespace AIMS {
    class Vehicle
    {
        private:
        float current_position_[3];
        ros::Subscriber pos_sub_;    
        ros::Publisher zyaw_pub_;

        public:
        Vehicle(ros::NodeHandle *nh) {
            current_position_[3] = {0, 0, 0};
            
            pos_sub_ = nh->subscribe("/red/pose", 100, &AIMS::Vehicle::pos_sub_callback, this);

            zyaw_pub_ = nh->advertise<geometry_msgs::PoseStamped>("/red/tracker/input_pose", 10);
        }

        void pos_sub_callback(const geometry_msgs::PointStamped &msg) {
            current_position_[0] = msg->pose.position.x;
            current_position_[1] = msg->pose.position.y;
            current_position_[2] = msg->pose.position.z;
        }

        void set_zoffset_yaw(float *target_poi_yaw) {

            EulerAngles angle;
            Quaternion q;
            angle.yaw = target_poi_yaw[3];
            q = ToQuaternion(angle);

            geometry_msgs::PointStamped zyaw_point_msg;
            zyaw_point_msg.pose.position.z = target_poi_yaw[2];
            zyaw_point_msg.pose.orientation.w = q.w;
            zyaw_point_msg.pose.orientation.x = q.x;
            zyaw_point_msg.pose.orientation.y = q.y;
            zyaw_point_msg.pose.orientation.z = q.z;
            zyaw_pub_.publish(zyaw_point_msg;
            ROS_INFO("Setting zoffset and yaw to target");
        }
    }
}



            
        