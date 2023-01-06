#include "slam_pioneer3dx/sensor.h"
#include "slam_pioneer3dx/motor.h"

#define TIME_STEP 32 //[ms]
//Global variables
ros::ServiceClient setTimeStepClient;
webots_ros::set_int setTimeStepSrv;
static int step = TIME_STEP;
void quit(int);
int main(int argc, char** argv)
{
    int stepMax = 1;
    int nStep = 0;
    //  look if a limit time was given
    if (argc < 2)
    {
        ROS_INFO("Usage: $ pioneer3dx [duration(seconds)].");
        return 1;
    }
    stepMax = atoll(argv[1]) * 1000/ TIME_STEP;
    ROS_INFO("Max step is %d.",stepMax);
    
    ros::init(argc, argv, "slam_pioneer3dx", ros::init_options::AnonymousName);
    ros::NodeHandle node;

    signal(SIGINT, quit);
    
    // wait for the 'ros' controller

    ros::service::waitForService("/pioneer3dx/robot/time_step");
    ros::spinOnce();

    // send robot time step to webots
    setTimeStepClient = node.serviceClient<webots_ros::set_int>("/pioneer3dx/robot/time_step");
    setTimeStepSrv.request.value = step;

    if (setTimeStepClient.call(setTimeStepSrv) && setTimeStepSrv.response.success)
        nStep++;
    else
        ROS_ERROR("Failed to call service time_step to update robot's time step.");
    Motor motor(&node);
    Sensor sensors(&node);
    motor.initializeMotor();
    sensors.initializeSensors();
    while (node.ok())
    {
        if (setTimeStepClient.call(setTimeStepSrv) && setTimeStepSrv.response.success)
            nStep++;
        else
            ROS_ERROR("Failed to call service time_step to update robot's time step.");
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