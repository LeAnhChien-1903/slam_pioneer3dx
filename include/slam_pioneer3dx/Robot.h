#ifndef ROBOT_H
#define ROBOT_H

#include "slam_pioneer3dx/device.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_broadcaster.h>
class Robot: public Devices
{
    private:
        ros::Subscriber imu_sub;
        ros::Subscriber gps_sub;
        ros::Subscriber lidar_sub;
        volatile float current_x, current_y, current_z;
        volatile float cur_rot_x, cur_rot_y, cur_rot_z, cur_rot_w;
    public:
        Robot(ros::NodeHandle *nodeHandle);
        void timerCallback();
        void imuCallback(const sensor_msgs::Imu &msg);
        void gpsCallback(const geometry_msgs::PointStamped &msg);
        void publishBaseLink();
        void publishLidarLink();
};
Robot::Robot(ros::NodeHandle *nodeHandle)
{
    node = *nodeHandle;
    initializeMotor();
    initialSensors();
    enablePointCloud();
    gps_sub = nodeHandle->subscribe("/pioneer3dx/gps/values", 100, &Robot::gpsCallback, this);
    imu_sub = nodeHandle->subscribe("/pioneer3dx/IU/quaternion", 100, &Robot::imuCallback, this);
}
void Robot::timerCallback()
{   
    std::cout<<"Current_x: "<<current_x<<std::endl;
    runMotor(1,1);
}
void Robot::gpsCallback(const geometry_msgs::PointStamped &msg)
{
    current_x = msg.point.x;
    current_y = msg.point.y;
    current_z = 0;  
}
void Robot::imuCallback(const sensor_msgs::Imu &msg)
{ 
    cur_rot_x = msg.orientation.x;
    cur_rot_y = msg.orientation.y;
    cur_rot_z = msg.orientation.z;
    cur_rot_w = msg.orientation.w;
}
void Robot::publishBaseLink()
{
    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(current_x, current_y, current_z));
    transform.setRotation(tf::Quaternion(cur_rot_x, cur_rot_y, cur_rot_z, cur_rot_w));
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/base_link"));
}
void Robot::publishLidarLink()
{
    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.1, 0, 0.195));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "/Lidar"));
}
#endif