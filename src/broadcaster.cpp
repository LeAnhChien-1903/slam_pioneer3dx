#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
// Create the node that will do the work of broadcasting the base_laser -> base_link transform over ROS
int main(int argc, char** argv)
{
    ros::init(argc, argv, "broadcaster");
    ros::NodeHandle node;
    ros::Rate rate(1);
    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    while (node.ok())
    {
        transform.setOrigin(tf::Vector3(0, 0, 0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/base_link"));
        rate.sleep();
    }
}