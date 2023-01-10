#ifndef DEVICE_H
#define DEVICE_H

#include <ros/ros.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_bool.h>
#include <webots_ros/robot_get_device_list.h>
#include <std_msgs/String.h>
#include <signal.h>
#include <stdio.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#define TIME_STEP 32
// Robot constants
#define WHEEL_BASE 0.165 // [m]
#define WHEEL_RADIUS 0.0975 // [m]
#define MAX_SPEED 12.3 // [rad/s]
#define LEFT 0
#define RIGHT 1

class Devices
{
    private:
        std::string robot_name = "/pioneer3dx";
        webots_ros::set_float setInfSrv;
        webots_ros::set_float setZeroSrv;
        std::vector<ros::ServiceClient> velocityClient;
        webots_ros::set_float leftWheelSrv;
        webots_ros::set_float rightWheelSrv;
        ros::ServiceClient setTimeStepClient;
        webots_ros::set_int setTimeStepSrv;
        std::vector<ros::ServiceClient> sensorsClient;
    protected:
        ros::NodeHandle node;
    public:
        void initializeMotor();// function for  initializing motors
        void runMotor(float omega_left, float omega_right); // run motor with speed in rad/s
        void initialSensors();// function for  initializing sensors
        void enablePointCloud(); // enable point cloud in webots 
};
void Devices::initializeMotor()
{
    setInfSrv.request.value = INFINITY; // for all positions of motors
    setZeroSrv.request.value = 0.0; // for all velocity of motors
    std::vector<ros::ServiceClient> motorClient;
    std::vector<std::string> actuators{"/left_wheel", "/right_wheel"};
    for (auto actuator = actuators.begin(); actuator != actuators.end(); ++actuator)
    {
        // setting position of actuators
        motorClient.push_back(node.serviceClient<webots_ros::set_float>(Devices::robot_name + *actuator + "/set_position"));
        ros::service::waitForService(Devices::robot_name + *actuator + "/set_position");
        motorClient.back().call(setInfSrv);
        // setting velocity  of actuators
        velocityClient.push_back(node.serviceClient<webots_ros::set_float>(Devices::robot_name + *actuator + "/set_velocity"));
        ros::service::waitForService(Devices::robot_name + *actuator + "/set_velocity");
        velocityClient.back().call(setZeroSrv);
    }
}
void Devices::runMotor(float omega_left, float omega_right)
{
    leftWheelSrv.request.value = omega_left;
    rightWheelSrv.request.value = omega_right;
    velocityClient[LEFT].call(leftWheelSrv);
    velocityClient[RIGHT].call(rightWheelSrv);
}
void Devices::initialSensors()
{
    setTimeStepSrv.request.value = TIME_STEP;
    std::vector<std::string> sensors{"/Lidar", "/IU", "/gps", "/right_wheel_sensor", "/left_wheel_sensor"}; // lists of sensors
    for (auto sensor = sensors.begin(); sensor != sensors.end(); ++sensor)
    {
        // service name
        sensorsClient.push_back(node.serviceClient<webots_ros::set_int>(Devices::robot_name + *sensor + "/enable"));
        ros::service::waitForService(Devices::robot_name + *sensor + "/enable");
        sensorsClient.back().call(setTimeStepSrv); // enable with the  TIME_STEP
    }
}
void Devices::enablePointCloud()
{
    ros::ServiceClient setPointCloudClient;
    webots_ros::set_bool setPointCloudSrv;
    setPointCloudClient = node.serviceClient<webots_ros::set_bool>("/pioneer3dx/Lidar/enable_point_cloud");
    ros::service::waitForService("/pioneer3dx/Lidar/enable_point_cloud");
    setPointCloudSrv.request.value = true;
    setPointCloudClient.call(setPointCloudSrv);
}

#endif