#include <ros/ros.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_float.h>
#include <sensor_msgs/Range.h>

#include <signal.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8MultiArray.h>
#include <cstdlib>

// Global defines
#define TRUE 1
#define FALSE 0
#define PI 3.14159265358979323846
#define SIMULATION 0
#define REALITY  2
#define TIME_STEP 64 // [ms]
#define WHEEL_BASE 0.165
#define WHEEL_RADIUS 0.0975
#define MAX_SPEED 12.3 // rad/s
#define LEFT 0
#define RIGHT 1
// ----------------------------------------
//          ROS CALLBACKS
//-----------------------------------------
ros::ServiceClient setTimeStepClient;
webots_ros::set_int setTimeStepSrv;
static int step = TIME_STEP;
void quit(int sig)
{
    setTimeStepSrv.request.value = 0;
    setTimeStepClient.call(setTimeStepSrv);
    ROS_INFO("User stopped the 'pioneer3dx' node.");
    ros::shutdown();
    exit(0);
}

// ----------------------------------------
//          BEHAVIORAL MODULES
//-----------------------------------------
int oam_active, oam_reset;
int oam_speed[2];
//-----------------------------------------
//          CONTROLLER
//-----------------------------------------
int main(int argc, char** argv )
{
    int i;
    int oamOfmSpeed[2];
    double speed[2];

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
    
    ros::init(argc, argv, "pioneer3dx", ros::init_options::AnonymousName);
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
    
    // set the motors to velocity control
    webots_ros::set_float wheelSrv;
    wheelSrv.request.value = INFINITY;
    ros::ServiceClient leftWheelPositionClient = node.serviceClient<webots_ros::set_float>("/pioneer3dx/left_wheel/set_position");
    leftWheelPositionClient.call(wheelSrv);
    ros::ServiceClient rightWheelPositionClient = node.serviceClient<webots_ros::set_float>("/pioneer3dx/right_wheel/set_position");
    rightWheelPositionClient.call(wheelSrv);
    ros::ServiceClient leftWheelVelocityClient = node.serviceClient<webots_ros::set_float>("//pioneer3dx/left_wheel/set_velocity");
    ros::ServiceClient rightWheelVelocityClient = node.serviceClient<webots_ros::set_float>("/pioneer3dx/right_wheel/set_velocity");

    if (setTimeStepClient.call(setTimeStepSrv) && setTimeStepSrv.response.success)
        nStep++;
    else
        ROS_ERROR("Failed to call service time_step to update robot's time step.");
    
    // main loop
    while (nStep < stepMax)
    {
        if (setTimeStepClient.call(setTimeStepSrv) && setTimeStepSrv.response.success)
            nStep++;
        else
            ROS_ERROR("Failed to call service time_step to update robot's time step.");
        // Speed initialization
        speed[LEFT] = 0.1*MAX_SPEED;
        speed[RIGHT] = 0.1*MAX_SPEED;
        wheelSrv.request.value = speed[LEFT];
        leftWheelVelocityClient.call(wheelSrv);
        wheelSrv.request.value = speed[RIGHT];
        rightWheelVelocityClient.call(wheelSrv);
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