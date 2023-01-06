#ifdef SENSOR_ENABLE_H
#define SENSOR_ENABLE_H

#include <ros/ros.h>
#include <webots_ros/Int32Stamped.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/robot_get_device_list.h>
#include <std_msgs/String.h>
#include <signal.h>
#include <stdio.h>
#include <iostream.h>
#include <geometry_msgs/Twist.h>

// Robot constants
#define TIME_STEP 32
#define WHEEL_BASE 0.165
#define WHEEL_RADIUS 0.0975

class SensorEnable
{
    private:
        ros::NodeHandle nh_;
        std::String robot_name;
        ros::Subscriber subscriber_name;
        ros::Subscriber subscriber_keyboard;
        webots_ros::set_int srv_timeStep;
        webots_ros::set_float srv_inf;
        webots_ros::set_float srv_zero;
        std::vector<ros::ServiceClient> vec_velocity;
        webots_ross::set_float srv_act;
        ros::Subscriber subscribe_cmd_vel;
        float linear_vel, angular_vel;
    public:
        SensorEnable(ros::NodeHandle* nodeHandle);
        void NameCallback(const std_msgs::String& msg);
        void KeyboardCallback(const webots_ros::Int32Stamped& msg);
        void CmdVelCallback(const geometry_msgs::Twist& msg);
        void Initialize_sensors();
        void teleop(int);
};
#endif