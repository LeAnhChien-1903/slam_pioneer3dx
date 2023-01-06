#include "slam_pioneer3dx/enable_sensor.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "slam_pioneer3dx", ros::init_options::AnonymousName);
    ros::NodeHandle node;
    SensorEnable robot(&node);
    robot.InitializeSensors();
    while (true)
    {
        robot.leftWheelSrv.request.value  =  1;
        robot.rightWheelSrv.request.value =  1;
        robot.velocityClient[LEFT].call(robot.leftWheelSrv);
        robot.velocityClient[RIGHT].call(robot.rightWheelSrv);
    }
    ros::spin();
    return 0;
}