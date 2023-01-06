#ifndef MOTOR_H
#define MOTOR_H

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
#define WHEEL_BASE 0.165 // [m]
#define WHEEL_RADIUS 0.0975 // [m]
#define MAX_SPEED 12.3 // [rad/s]
#define LEFT 0
#define RIGHT 1
class Motor
{
    private:
        ros::NodeHandle node;
        std::string robot_name = "pioneer3dx";
        webots_ros::set_float setInfSrv;
        webots_ros::set_float setZeroSrv;
        std::vector<ros::ServiceClient> velocityClient;
        webots_ros::set_float leftWheelSrv;
        webots_ros::set_float rightWheelSrv;
        ros::Subscriber cmd_vel_sub;
        float linear_vel, angular_vel;
    public:
        Motor(ros::NodeHandle*  node);
        void velocityCallback(const geometry_msgs::Twist&  msg);
        void initializeMotor();// function for  initializing motors
        void runMotor(float omega_left, float omega_right); // run motor with speed in rad/s
};
Motor::Motor(ros::NodeHandle * nodeHandle):node(*nodeHandle)
{
    setInfSrv.request.value = INFINITY; // for all positions of motors
    setZeroSrv.request.value = 0.0; // for all velocity of motors
}
void Motor::velocityCallback(const geometry_msgs::Twist &msg)
{
    linear_vel = msg.linear.x;
    angular_vel = msg.angular.z;
    leftWheelSrv.request.value = (linear_vel - angular_vel*WHEEL_BASE)/WHEEL_RADIUS;
    rightWheelSrv.request.value = (linear_vel + angular_vel*WHEEL_BASE)/WHEEL_RADIUS;
    velocityClient[LEFT].call(leftWheelSrv);
    velocityClient[RIGHT].call(rightWheelSrv);
}
void Motor::initializeMotor()
{
    std::vector<ros::ServiceClient> motorClient;
    std::vector<std::string> actuators{"/left_wheel", "/right_wheel"};
    for (auto actuator = actuators.begin(); actuator != actuators.end(); ++actuator)
    {
        // setting position of actuators
        motorClient.push_back(node.serviceClient<webots_ros::set_float>(Motor::robot_name + *actuator + "/set_position"));
        ros::service::waitForService(Motor::robot_name + *actuator + "/set_position");
        motorClient.back().call(setInfSrv);
        // setting velocity  of actuators
        velocityClient.push_back(node.serviceClient<webots_ros::set_float>(Motor::robot_name + *actuator + "/set_velocity"));
        ros::service::waitForService(Motor::robot_name + *actuator + "/set_velocity");
        velocityClient.back().call(setZeroSrv);
    }
}   

void Motor::runMotor(float omega_left, float omega_right)
{
    leftWheelSrv.request.value = omega_left;
    rightWheelSrv.request.value = omega_right;
    velocityClient[LEFT].call(leftWheelSrv);
    velocityClient[RIGHT].call(rightWheelSrv);
}
#endif 