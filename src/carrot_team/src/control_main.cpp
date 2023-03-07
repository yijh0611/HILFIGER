#include <iostream>
#include <vector>
#include <cmath>

#include "ros/ros.h"
#include "carrot_team/poi.h"
#include "geometry_msgs/PoseStamped.h"

#include "orientation.hpp"
#include "carrot_team/class.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_main");
    ros::NodeHandle nh;

    Target_POI target_poi = Target_POI(&nh);
    AIMS::Vehicle carrot_vehicle = AIMS::Vehicle(&nh);
    ros::spin();
}