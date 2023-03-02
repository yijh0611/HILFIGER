#include <iostream>
#include <cmath>
#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "carrot_team/poi.h"

using namespace std;

// Global variables
float current_pos[3];
vector<bool> waypoint_index;

// ------------------------Orientation Definition--------------------------
struct Quaternion {
    float w, x, y, z;
};

struct EulerAngles {
    float roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q)
{
    EulerAngles angles;

    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    float cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

Quaternion ToQuaternion(EulerAngles angles) // roll (x), pitch (Y), yaw (z)
{
    // Abbreviations for the various angular functions
    float cr = cos(angles.roll * 0.5);
    float sr = sin(angles.roll * 0.5);
    float cp = cos(angles.pitch * 0.5);
    float sp = sin(angles.pitch * 0.5);
    float cy = cos(angles.yaw * 0.5);
    float sy = sin(angles.yaw * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}


// ------------------------Class Definition--------------------------
namespace carrot_team {

    class Drone {
        public:
            Drone();
            void go_to_poi(float x, float y, float z, float yaw);

        private:
            ros::NodeHandle _nh;
            float _x, _y, _z, _yaw;
            ros::Publisher _trajectory_point_pub;
    };
}

carrot_team::Drone::Drone() : _x(0), _y(0), _z(0), _yaw(0)
{
    _trajectory_point_pub = _nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("red/position_hold/trajectory", 1000);
}

void carrot_team::Drone::go_to_poi(float x, float y, float z, float yaw)
{
    _x = x;
    _y = y;
    _z = z;
    _yaw = yaw;

    EulerAngles angles;
    Quaternion q;
    angles.yaw = _yaw;
    q = ToQuaternion(angles);

    trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;
    trajectory_point_msg.transforms.resize(1);
    trajectory_point_msg.velocities.resize(1);
    trajectory_point_msg.accelerations.resize(1);

    trajectory_point_msg.transforms[0].translation.x = _x;
    trajectory_point_msg.transforms[0].translation.y = _y;
    trajectory_point_msg.transforms[0].translation.z = _z;
    trajectory_point_msg.transforms[0].rotation.x = q.x;
    trajectory_point_msg.transforms[0].rotation.y = q.y;
    trajectory_point_msg.transforms[0].rotation.z = q.z;
    trajectory_point_msg.transforms[0].rotation.w = q.w;
    _trajectory_point_pub.publish(trajectory_point_msg);
}

// ---------------------------Call back function--------------------------
void current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pos[0] = msg->pose.position.x;
    current_pos[1] = msg->pose.position.y;
    current_pos[2] = msg->pose.position.z;
}

void calculate_range(float *current, float *target, float *range, float *yaw)
{
    float delta_x, delta_y, delta_z;
    delta_x = current[0] - target[0];
    delta_y = current[1] - target[1];
    delta_z = current[2] - target[2];
    *range = (delta_x * delta_x) + (delta_y * delta_y) + (delta_z * delta_z);
    *yaw = atan(delta_x / delta_y);
}

void find_target_poi_callback(const carrot_team::poi::ConstPtr &msg)
{
    int poi_len = msg->poi.size();
    waypoint_index.resize(poi_len);
    float range, yaw;
    float min_range = 10e9f;
    float temp_poi[3];
    int i;

    // find minimum range in pois
    for (i = 0; i < poi_len; ++i)
    {
        temp_poi[0] = msg->poi[i].x;
        temp_poi[1] = msg->poi[i].y;
        temp_poi[2] = msg->poi[i].z;
        calculate_range(current_pos, temp_poi, &range, &yaw);

        if (min_range >= range)
        {
            min_range = range;
            waypoint_index[i] = true;
        }
    }
    cout << range << "\n"
         << yaw   << "\n"
         << waypoint_index[i] << endl;

    carrot_team::Drone carrot_drone;
    carrot_drone.go_to_poi(msg->poi[i].x, msg->poi[i].y, msg->poi[i].z, yaw); 
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_to_poi");
    ros::NodeHandle nh;

    ros::Subscriber current_pos_sub = nh.subscribe("red/pose", 10, &current_pose_callback);
    ros::Subscriber poi_arrary_sub = nh.subscribe("poi", 10, &find_target_poi_callback);

    ROS_INFO("vehicle starts!!");
    ros::spin();
    return 0;
}