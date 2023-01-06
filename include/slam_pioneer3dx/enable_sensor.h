#ifndef ENABLE_SENSOR_H
#define ENABLE_SENSOR_H

#include <ros/ros.h>
#include <webots_ros/Int32Stamped.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/robot_get_device_list.h>
#include <std_msgs/String.h>
#include <signal.h>
#include <stdio.h>
#include <iostream>
#include <geometry_msgs/Twist.h>

// Robot constants
#define TIME_STEP 32 // [ms]
#define WHEEL_BASE 0.165 // [m]
#define WHEEL_RADIUS 0.0975 // [m]
#define MAX_SPEED 12.3 // [rad/s]
#define LEFT 0
#define RIGHT 1
class SensorEnable
{
    private:
        ros::NodeHandle node;
        std::string robot_name = "pioneer3dx";
        ros::Subscriber name_sub;
        ros::Subscriber keyboard_sub;
        ros::ServiceClient setTimeStepClient;
        webots_ros::set_int setTimeStepSrv;
        webots_ros::set_float setInfSrv;
        webots_ros::set_float setZeroSrv;
        std::vector<ros::ServiceClient> velocityClient;
        webots_ros::set_float leftWheelSrv;
        webots_ros::set_float rightWheelSrv;
        ros::Subscriber cmd_vel_sub;
        float linear_vel, angular_vel;
    public:
        SensorEnable(ros::NodeHandle*  node);
        void nameCallback(const std_msgs::String& msg);
        void keyboardCallback(const webots_ros::Int32Stamped& msg);
        void velocityCallback(const geometry_msgs::Twist&  msg);
        void InitializeSensors();// function for  initializing sensors
        void teleop(int); // Function for teleop 
};
#include "slam_pioneer3dx/enable_sensor.h"
SensorEnable::SensorEnable(ros::NodeHandle * nodeHandle)
{
    node = *nodeHandle;
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
    velocityClient[LEFT].call(leftWheelSrv);
    velocityClient[RIGHT].call(rightWheelSrv);
}
void SensorEnable::nameCallback(const std_msgs::String& msg)
{
    InitializeSensors();
}
void SensorEnable::InitializeSensors()
{
    std::vector<std::string> sensors{"/Lidar", "/IU", "/keyboard","/so0", "/so1", "/so2", 
                                    "/so3", "/so4", "/so5", "/so6", "/so7",
                                    "/right_wheel_sensor", "/left_wheel_sensor"};
    std::vector<ros::ServiceClient> sensorsClient;
    for (auto sensor = sensors.begin();   sensor != sensors.end(); ++sensor)
    {
        // service name
        sensorsClient.push_back(node.serviceClient<webots_ros::set_int>(SensorEnable::robot_name + *sensor + "/enable"));
        ros::service::waitForService(SensorEnable::robot_name + *sensor + "/enable");
        sensorsClient.back().call(setTimeStepSrv); // enable with the  TIME_STEP
    }
    keyboard_sub = node.subscribe(SensorEnable::robot_name + "/keyboard/key", 1, &SensorEnable::keyboardCallback, this);
    std::vector<std::string> actuators{"/left_wheel", "/right_wheel"};
    for (auto actuator = actuators.begin(); actuator != actuators.end(); ++actuator)
    {
        // setting position of actuators
        sensorsClient.push_back(node.serviceClient<webots_ros::set_float>(SensorEnable::robot_name + *actuator + "/set_position"));
        ros::service::waitForService(SensorEnable::robot_name + *actuator + "/set_position");
        sensorsClient.back().call(setInfSrv);
        // setting velocity  of actuators
        velocityClient.push_back(node.serviceClient<webots_ros::set_float>(SensorEnable::robot_name + *actuator + "/set_velocity"));
        ros::service::waitForService(SensorEnable::robot_name + *actuator + "/set_velocity");
        velocityClient.back().call(setZeroSrv);
    }
}   
void SensorEnable::keyboardCallback(const webots_ros::Int32Stamped& msg)
{
    teleop(msg.data);
}
void SensorEnable::teleop(int key)
{
    // UP 315
    // DOWN 317
    // LEFT 314
    // RIGHT 316
    switch(key)
    {
        case 315:
        {
            leftWheelSrv.request.value = leftWheelSrv.request.value + 0.1;
            rightWheelSrv.request.value = rightWheelSrv.request.value + 0.1;
            if (leftWheelSrv.request.value > MAX_SPEED) leftWheelSrv.request.value =  MAX_SPEED;
            if (rightWheelSrv.request.value > MAX_SPEED) rightWheelSrv.request.value = MAX_SPEED;
            break;
        }
        case 317:
        {
            leftWheelSrv.request.value = leftWheelSrv.request.value - 0.1;
            rightWheelSrv.request.value = rightWheelSrv.request.value - 0.1;
            if (leftWheelSrv.request.value < -MAX_SPEED) leftWheelSrv.request.value  = -MAX_SPEED;
            if (rightWheelSrv.request.value < -MAX_SPEED) rightWheelSrv.request.value = -MAX_SPEED;
            break;
        }
        case 314:
        {
            leftWheelSrv.request.value = leftWheelSrv.request.value - 0.1;
            if  (leftWheelSrv.request.value < -MAX_SPEED) leftWheelSrv.request.value = -MAX_SPEED;
            break;
        }
        case 316:
        {
            rightWheelSrv.request.value = rightWheelSrv.request.value - 0.1;
            if (rightWheelSrv.request.value < -MAX_SPEED) rightWheelSrv.request.value = -MAX_SPEED;
            break;
        }
        default:
        {
            break;
        }
    }
    velocityClient[LEFT].call(leftWheelSrv);
    velocityClient[RIGHT].call(rightWheelSrv);
    std::cout<<key<<std::endl;
}
#endif 