/*******************************************************************************
 * @file control.cpp
 * @author James Anderson <jra798>
 * @version 1.1
 * @date 1/21/12
 * @brief controlls which output goes to motors 
 ******************************************************************************/


/***********************************************************
* ROS specific includes
***********************************************************/
#include "ros/ros.h"

/***********************************************************
* Message includes
***********************************************************/
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <wiimote/State.h>
#include <wiimote/LEDControl.h>
#include <wiimote/RumbleControl.h>
#include <sound_play/SoundRequest.h>
#include <wiimote/TimedSwitch.h>
#include <MST_Estop/Control_State.h>
#include <MST_Estop/Estop_State.h>
#include <MST_Position/Target_Heading.h>

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

ros::Subscriber                 nav_sub;
ros::Subscriber                 wiimote_state_sub;
ros::Subscriber                 estop_sub;
ros::Subscriber                 pos_sub;


/***********************************************************
* Publishers
***********************************************************/

ros::Publisher                  wiimote_led_pub;
ros::Publisher                  wiimote_rum_pub;
ros::Publisher                  motor_pub;
ros::Publisher                  sound_pub;
ros::Publisher                  estop_pub;



/***********************************************************
* Global variables
***********************************************************/

//enumorator for robot mode
enum 
{
    standby,
    wiimote_mode,
    autonomous,
    jaus
}   Mode;

Mode mode;

//enumerator for the autonomous mode
enum
{
    navigation,
    autonomous_waypoints,
    carrot
}   autonmous_mode;

//storage for wiimote toggle bools 
//last one is for the disconect and starts empty
bool wii_togg[15] = { false,false,false,false,false,
                      false,false,false,false,false,
                      false,false,false,false,false } ;



bool                            wiimote_init;
bool                            robot_init;

                


ros::Time                       home_press_begin;

MST_Estop::Estop_State          estop;

geometry_msgs::Twist            wii_twist;
geometry_msgs::Twist            nav_twist;
bool                            wii_updated;
bool                            nav_updated;

bool                            wii_togg_one;
bool                            wii_togg_two;
bool                            wii_togg_up;
bool                            wii_togg_down;
bool                            wii_togg_left;
bool                            wii_togg_right;
bool                            wii_togg_plus;
bool                            wii_togg_minus;
bool                            wii_togg_home;
bool                            wii_togg_a;
bool                            wii_dis;
bool                            estop_togg = 0;
bool                            done_togg = 0;


MST_Control::Control_ParamsConfig params;
MST_Position::Target_Heading  target;


/***********************************************************
* Namespace Changes
***********************************************************/
namespace enc = sensor_msgs::image_encodings;
using namespace std;


