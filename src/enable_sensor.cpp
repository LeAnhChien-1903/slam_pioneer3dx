#include "slam_pioneer3dx/enable_sensor.h"

SensorEnable::SensorEnable(ros::NodeHandle * nodeHandle):node(*nodeHandle)
{
    setTimeStepSrv.request.value = TIME_STEP;// for all sensors
    setInfSrv.request.value = INFINITY; // for all positions of motors
    setZeroSrv.request.value = 0.0; // for all velocity of motors
    std::cout<<setTimeStepSrv.request.value<<std::endl;
    name_sub = node.subscribe("/model_name", 1, &SensorEnable::nameCallback, this);
    cmd_vel_sub = node.subscribe("/cmd_vel", 1, &SensorEnable::velocityCallback, this);

}
void SensorEnable::velocityCallback(const geometry_msgs::Twist &msg)
{
    linear_vel = msg.linear.x;
    angular_vel = msg.angular.z;
    leftWheelSrv.request.value = (linear_vel - angular_vel*WHEEL_BASE)/WHEEL_RADIUS;
    rightWheelSrv.request.value = (linear_vel + angular_vel*WHEEL_BASE)/WHEEL_RADIUS;
    velocityClient[0].call(leftWheelSrv);
    velocityClient[1].call(rightWheelSrv);
    
}