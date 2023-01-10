#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <ros/ros.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include <webots_ros/robot_get_device_list.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <cmath>

class Localization
{
    private:
        ros::NodeHandle node;
        std::string robot_name;
        ros::Subscriber subscribe_imu, subscribe_gps;
        
    public:
        float current_x, current_y, current_z;
        float current_rot_x, current_rot_y, current_rot_z, current_rot_w;
        float roll, pitch, yaw;
        Localization(ros::NodeHandle *nodeHandle);
        void imuCallback(const sensor_msgs::Imu &msg);
        void gpsCallback(const geometry_msgs::PointStamped &msg);
        void getIMU();
        void getGPS();
        void publish_base_link();
        void ToEulerAngles(float x, float y, float z, float w);
        void publish_lidar_link();
};
Localization::Localization(ros::NodeHandle *nodeHandle):node(*nodeHandle)
{
    robot_name = "/pioneer3dx";
    getGPS();
    getIMU();
}
void Localization::imuCallback(const sensor_msgs::Imu &msg)
{
    current_rot_x = msg.orientation.x;
    current_rot_y = msg.orientation.y;
    current_rot_z = msg.orientation.z;
    current_rot_w = msg.orientation.w;
    publish_base_link();
    publish_lidar_link();
}
void Localization::gpsCallback(const geometry_msgs::PointStamped &msg)
{
    current_x = msg.point.x;
    current_y = msg.point.y;
    current_z = msg.point.z;
}
void Localization::getIMU()
{
    subscribe_imu = node.subscribe("/pioneer3dx/IU/quaternion", 10, &Localization::imuCallback, this);
}
void Localization::getGPS()
{
    subscribe_gps = node.subscribe("/pioneer3dx/gps/values", 10, &Localization::gpsCallback, this);

}
void Localization::ToEulerAngles(float x, float y, float z, float w)
{
    // roll (x-axis rotation)
    double sinR_cosP = 2 * (w * x + y * z);
    double cosR_cosP = 1 - 2 * (x * x + y * y);
    roll = std::atan2(sinR_cosP, cosR_cosP);

    // pitch (y-axis rotation)
    double sinP = 2 * (w * y - z * x);
    if (std::abs(sinP) >= 1)
        pitch = std::copysign(M_PI / 2, sinP); // use 90 degrees if out of range
    else
        pitch = std::asin(sinP);

    // yaw (z-axis rotation)
    double sinY_cosP = 2 * (w * z + x * y);
    double cosY_cosP = 1 - 2 * (y * y + z * z);
    yaw = std::atan2(sinY_cosP, cosY_cosP);
}
void Localization::publish_base_link()
{
    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(current_x, current_y, current_z));
    transform.setRotation(tf::Quaternion(current_rot_x, current_rot_y, current_rot_z, current_rot_w));
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/base_link"));
}
void Localization::publish_lidar_link()
{
    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.1, 0, 0.195));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "/Lidar"));
}
#endif