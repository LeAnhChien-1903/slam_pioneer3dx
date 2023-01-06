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
// Distance sensor
#define numOfDistanceSensors 8
#define so0 0;
#define so1 1;
#define so2 2;
#define so3 3;
#define so4 4;
#define so5 5;
#define so6 6;
#define so7 7;
class Sensor
{
    private:
        ros::NodeHandle node;
        std::string robot_name = "pioneer3dx";
        ros::ServiceClient setTimeStepClient;
        webots_ros::set_int setTimeStepSrv;
    public:
        Sensor(ros::NodeHandle*  nodeHandle);
        void initializeSensors();// function for  initializing sensors
};
Sensor::Sensor(ros::NodeHandle * nodeHandle):node(*nodeHandle)
{
}
void Sensor::initializeSensors()
{
    std::vector<std::string> sensors{"/Lidar", "/IU", "/keyboard","/so0", "/so1", "/so2", 
                                    "/so3", "/so4", "/so5", "/so6", "/so7",
                                    "/right_wheel_sensor", "/left_wheel_sensor"};
    std::vector<ros::ServiceClient> sensorsClient;
    for (auto sensor = sensors.begin();   sensor != sensors.end(); ++sensor)
    {
        // service name
        sensorsClient.push_back(node.serviceClient<webots_ros::set_int>(Sensor::robot_name + *sensor + "/enable"));
        ros::service::waitForService(Sensor::robot_name + *sensor + "/enable");
        sensorsClient.back().call(setTimeStepSrv); // enable with the  TIME_STEP
    }
}   
#endif 