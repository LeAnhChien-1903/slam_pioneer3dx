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
        void keyboardCallback(const std_msgs::String& msg);
        void velocityCallback(const geometry_msgs::Twist&  msg);
        void InitializeSensor();// function for  initializing sensors
        void teleop(int); // Function for teleop 
        void quit(int);
};
#endif 