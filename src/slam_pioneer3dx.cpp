#include "slam_pioneer3dx/enable_sensor.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "slam_pioneer3dx");
    ros::NodeHandle node;
    SensorEnable member(&node);
    ros::spin();
    return 0;
}