/*******************************************************************************
 * @file control.h
 * @author Michael Lester
 * @version 1.1
 * @date 4/3/13
 * @brief controlls which output goes to motors 
 * @Re-made for xbox_Mode
 ******************************************************************************/
 
 /***********************************************************
* ROS specific includes
***********************************************************/
#include "ros/ros.h"
/***********************************************************
* Message includes
***********************************************************/
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sound_play/SoundRequest.h>
#include <MST_Estop/Estop_State.h>
#include <MST_Position/Target_Heading.h>
#include "mst_midg/IMU.h"
#include "sensor_msgs/Joy.h"
/***********************************************************
* Other includes
***********************************************************/
#include <iostream>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dynamic_reconfigure/server.h>
#include <MST_Control/Control_ParamsConfig.h>
/***********************************************************
* Subscribers
***********************************************************/
ros::Subscriber                 xbox_state_sub;
ros::Subscriber                 s_msg;
ros::Subscriber                 nav_sub;
ros::Subscriber                 estop_sub;
ros::Subscriber                 pos_sub;


/***********************************************************
* Publishers
***********************************************************/


//ros::Publisher                  wiimote_rum_pub; Switch out for xbox rumble
ros::Publisher                  motor_pub;
ros::Publisher                  p_cmd_vel;
ros::Publisher                  sound_pub;
ros::Publisher                  estop_pub;



/***********************************************************
* Global variables
***********************************************************/

/*-----------------------------------
	Velocity and sensor data variables
	-----------------------------------*/
    double m_linear; //These are for simulation
    double m_angular;
    //xbox movement x=linear y=angular
    float  joy_rightstick_x;
    float  joy_rightstick_y;
    float  joy_leftstick_x; 
    float  joy_leftstick_y;
    float  joy_r_trigger;
    float  joy_l_trigger;
    //xbox buttons to assign cmds
    #define  joy_button_A   0
    #define  joy_button_B   1
    #define  joy_button_X   2
    #define  joy_button_Y   3
    #define  joy_r_button   5
    #define  joy_l_button   4
    #define  joy_start_b    7
    #define  joy_back_b     6
    #define  joy_dpad_up    13
    #define  joy_dpad_dwn   14
    #define  joy_dpad_l     11
    #define  joy_dpad_r     12
    #define  joy_light      8
    #define  joy_l_stick    9
    #define  joy_r_stick    10static bool check_togg(bool, int);static bool check_togg(bool, int);
    
//Enumorator for each mode 
enum
Mode
{
    standby,
    xbox_mode,
    autonomous
} 
mode_;

//Enumerator for autonomous mode
enum
Autonomous_Mode
{
    navigation,
    autonomous_waypoints
};
Autonomous_Mode autonomous_mode;

MST_Control::Control_ParamsConfig params;

geometry_msgs::Twist nav_twist;           //Autonomous Navigation

/***************************************************************** 
*This is where we changed it from wii_twist to geometry_twist.
*If using a different remote in the future, assign geometry_twist
*so we have a universal twist for any controller
*
*See xbox_callback
******************************************************************/

geometry_msgs::Twist geometry_twist;      
bool xtogg[30];
bool robot_init;
bool done_togg = 0;
bool estop_togg = 0;
int last_msg_waypoint = 0;

/***********************************************************
* Namespace Changes
***********************************************************/
namespace enc = sensor_msgs::image_encodings;
using namespace std;

/***********************************************************
* Function prototypes
***********************************************************/
void change_mode(Mode new_mode);
void say(std::string );
void play(std::string );
void stop_robot();
bool check_togg(bool, int);
    

