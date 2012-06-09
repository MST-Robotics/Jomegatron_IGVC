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
#include <MST_Estop/Estop_State.h>
#include <MST_Position/Target_Heading.h>
#include "MST_JAUS/JAUS_in.h"
#include "MST_JAUS/JAUS_out.h"

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
* Defines
***********************************************************/
#define MSG_BTN_1      0
#define MSG_BTN_2      1
#define MSG_BTN_A      4
#define MSG_BTN_B      5
#define MSG_BTN_PLUS   2
#define MSG_BTN_MINUS  3
#define MSG_BTN_LEFT   6
#define MSG_BTN_RIGHT  7
#define MSG_BTN_UP     8
#define MSG_BTN_DOWN   9
#define MSG_BTN_HOME   10

//start at +11 in togg
#define MSG_BTN_Z      0
#define MSG_BTN_C      1

//start at +13 in togg
#define MSG_CLASSIC_BTN_X        0
#define MSG_CLASSIC_BTN_Y        1
#define MSG_CLASSIC_BTN_A        2
#define MSG_CLASSIC_BTN_B        3
#define MSG_CLASSIC_BTN_PLUS     4
#define MSG_CLASSIC_BTN_MINUS    5
#define MSG_CLASSIC_BTN_LEFT     6
#define MSG_CLASSIC_BTN_RIGHT    7
#define MSG_CLASSIC_BTN_UP       8
#define MSG_CLASSIC_BTN_DOWN     9
#define MSG_CLASSIC_BTN_HOME     10
#define MSG_CLASSIC_BTN_L        11
#define MSG_CLASSIC_BTN_R        12
#define MSG_CLASSIC_BTN_ZL       13
#define MSG_CLASSIC_BTN_ZR       14


/***********************************************************
* Subscribers
***********************************************************/

ros::Subscriber                 nav_sub;
ros::Subscriber                 wiimote_state_sub;
ros::Subscriber                 jaus_sub;
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
ros::Publisher                  jaus_pub;



/***********************************************************
* Global variables
***********************************************************/

//enumorator for robot mode
enum 
Mode
{
    standby,
    wiimote_mode,
    autonomous,
    jaus
};

Mode mode_;

//enumerator for the autonomous mode
enum
Autonmous_Mode
{
    navigation,
    autonomous_waypoints,
    carrot
};

Autonmous_Mode autonmous_mode;

//enumerator for JAUS mode
enum
JAUS_Mode
{
    jaus_resume,
    jaus_standby,
    jaus_shutdown
};

JAUS_Mode jaus_mode;

//storage for wiimote toggle bools 
//last one is for the disconect and starts empty
bool wii_togg[30] ;



bool                            wiimote_init;
bool                            robot_init;

                


ros::Time                       home_press_begin;

MST_Estop::Estop_State          estop;

geometry_msgs::Twist            wii_twist;
geometry_msgs::Twist            nav_twist;
MST_JAUS::JAUS_in               jaus_msg;


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

/***********************************************************
* Function prototypes
***********************************************************/
static void change_mode(Mode new_mode);
static void say(std::string );
static void play(std::string );
static void stop_robot();
static bool check_togg(bool, int);


