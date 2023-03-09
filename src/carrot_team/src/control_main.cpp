#include <iostream>
#include <vector>
#include <cmath>

#include "ros/ros.h"
#include "carrot_team/poi.h"
#include "geometry_msgs/PoseStamped.h"

#include "carrot_team/orientation.hpp"
#include "carrot_team/class.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_main");
    ros::NodeHandle nh;

    bool where = true;
    float current_position[3];  // x, y, z
    float target_poi_yaw[4];    // x, y, z, yaw

    AIMS::Vehicle carrot_vehicle = AIMS::Vehicle(&nh);
    Target_POI target_poi = Target_POI(&nh);
    Depth depth = Depth(&nh);
    ros::Rate rate(1);
    while (ros::ok()) {
        if (where) {
            // Request: I'm here, where to go?
            carrot_vehicle.get_current_pos(current_position);

            // Response: Find the target poi
            target_poi.find_min_range(current_position);
            target_poi.get_target_poi(target_poi_yaw);

            // Control
            carrot_vehicle.set_zoffset_yaw(target_poi_yaw);
            where = false;
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}