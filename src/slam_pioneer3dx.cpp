#include "slam_pioneer3dx/sensor.h"
#include "slam_pioneer3dx/motor.h"

//Global variables
ros::ServiceClient setTimeStepClient;
webots_ros::set_int setTimeStepSrv;
static int step = TIME_STEP;
void quit(int);
int main(int argc, char** argv)
{
    ros::init(argc, argv, "slam_pioneer3dx", ros::init_options::AnonymousName);
    ros::NodeHandle node;

    signal(SIGINT, quit);
    Sensor sensors(&node);
    Motor motor(&node);
    sensors.initializeSensors();
    motor.initializeMotor();
    while (node.ok())
    {
        motor.runMotor(1, 1);
    }
    // tells Webots this node will stop using time_step service
    setTimeStepSrv.request.value = 0;
    if (setTimeStepClient.call(setTimeStepSrv) && setTimeStepSrv.response.success)
        ROS_INFO("Robot's time step updated to end simulation.");
    else
        ROS_ERROR("Failed to call service time_step to end simulation.");
    ros::shutdown();
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