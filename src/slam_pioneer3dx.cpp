#include "slam_pioneer3dx/Robot.h"
//Global variables
ros::ServiceClient setTimeStepClient;
webots_ros::set_int setTimeStepSrv;
float speed = 1;
static int step = TIME_STEP;
void quit(int);
int main(int argc, char** argv)
{
    ros::init(argc, argv, "slam_pioneer3dx", ros::init_options::AnonymousName);
    ros::NodeHandle node;

    signal(SIGINT, quit);
    Robot robot(&node);
    ros::Timer timer = node.createTimer(ros::Duration(1.0 / 10.0), std::bind(&Robot::timerCallback,robot));
    ros::spin();
    return 0;
}
void quit(int sig)
{
    setTimeStepSrv.request.value = 0;
    setTimeStepClient.call(setTimeStepSrv);
    ROS_INFO("User stopped the 'pioneer3dx' node.");
    ros::shutdown();
    exit(0);
}
