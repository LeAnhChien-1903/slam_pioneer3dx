#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_tf_broadcaster");
    ros::NodeHandle node;

    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Rate rate(10.0);
    while (node.ok())
    {
        /*
            Create a new transform, from the parent turtle1 to the new child carrot1. The carrot1 frame is 2 meters
            offset to the left from the turtle1
        */
        transform.setOrigin(tf::Vector3(2.0*sin(ros::Time::now().toSec()), 2.0*cos(ros::Time::now().toSec()), 0.0));
        transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/base_link"));
        rate.sleep();
    }
    return 0;
}