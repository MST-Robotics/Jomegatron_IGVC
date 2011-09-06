/*******************************************************************************
 * @file control.cpp
 * @author James Anderson <jra798>
 * @version 1.0
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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dynamic_reconfigure/server.h>
#include <MST_Control/Control_ParamsConfig.h>


/***********************************************************
* Global variables
***********************************************************/


ros::Subscriber                 nav_sub;
ros::Subscriber                 wiimote_state_sub;
ros::Subscriber                 estop_sub;
ros::Publisher                  wiimote_led_pub;
ros::Publisher                  wiimote_rum_pub;
ros::Publisher                  motor_pub;
ros::Publisher                  sound_pub;
ros::Publisher                  estop_pub;
ros::Subscriber                  pos_sub;


bool                            wiimote_init;
bool                            robot_init;

                
std::string                     mode;
std::string                     autonmous_mode;

ros::Time                       home_press_begin;

MST_Estop::Estop_State          estop;

geometry_msgs::Twist            wii_twist;
geometry_msgs::Twist            nav_twist;
bool                            wii_updated;
bool                            nav_updated;

bool 					        wii_togg_one;
bool 							wii_togg_two;
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
* Function prototypes
***********************************************************/
void display_mode();
void say(std::string );
void play(std::string );
void stop_robot();


/***********************************************************
* Namespace Changesjames@james-laptop:~/Documents/Joemegatron_IGVC_2011/ROS_test/Midg/src$ 

***********************************************************/
namespace enc = sensor_msgs::image_encodings;
using namespace std;

/***********************************************************
* Message Callbacks
***********************************************************/

void pos_callback( const MST_Position::Target_Heading::ConstPtr& msg)
{
		if(msg->done && !done_togg)
		{
			play(params.done_sound);
			ros::Duration(6).sleep();
			mode = "standby";
			display_mode();
			done_togg = true;
		}
	    else if (!msg->done)
	    {
	    	done_togg = false;
	    }
		//waypoint 

}

/***********************************************************
* @fn wiimote_callback(const wiimote::State::ConstPtr& state
* @brief handels input from the wiimote
* @pre takes in a wiimote state message
* @post computes wii_twist variable and changes mode
***********************************************************/
void wiimote_callback(const wiimote::State state)
{
	//sets inital conection and resets if node restarted
	//when wiimote is killed and restarted it continuously
	// publishes its last message the checking of home handels this	
	if(wiimote_init && (!state.buttons[state.MSG_BTN_HOME] && wii_dis))
	{
		ROS_INFO("Control: Wiimote Conected");
		
		wiimote::RumbleControl rumble;
		//makes wiimote rumble 3 times
		rumble.rumble.switch_mode = rumble.rumble.REPEAT;
		rumble.rumble.num_cycles = 3;
		rumble.rumble.set_pulse_pattern_size(2); 
		rumble.rumble.pulse_pattern[0] = .25;
		rumble.rumble.pulse_pattern[1] = .5;
		
		ros::Duration(3).sleep();
		wiimote_rum_pub.publish(rumble);
		
		say("wiimote connected. Joe-Mega-Tron standing by");
		
		mode = "standby";
		display_mode();
		
		wii_dis = false;
		wiimote_init = false;
	}
	
	//figure out mode
	if(mode == "standby" || mode == "jaus")
	{
		if(state.buttons[state.MSG_BTN_1] && !wii_togg_one)
		{
			mode = "wiimote";
			display_mode();
			wii_togg_one = true;
		}
	    else if (!state.buttons[state.MSG_BTN_1])
	    {
	    	wii_togg_one = false;
	    }
		if(state.buttons[state.MSG_BTN_2] && !wii_togg_two)
		{
			mode = "autonomous";
			display_mode();
			wii_togg_two = true;		
		}
		else if (!state.buttons[state.MSG_BTN_2])
		{
			wii_togg_two = false;
		}
		
	}
	
	//plus an minus sounds plus and minus ar mixed up with 
	//a and b in the the apis map
	if(state.buttons[2] && !wii_togg_plus)
	{
		say(params.plus_mesage);
		wii_togg_plus = true;
	}
	else if(!state.buttons[2])
	{
		wii_togg_plus = false;
	}
	if(state.buttons[3] && !wii_togg_minus)
	{
        play(params.minus_sound);
		wii_togg_minus = true;
	}
	else if(!state.buttons[3])

	{
		wii_togg_minus = false;
	}
	
	//shut down code on 
	if(state.buttons[state.MSG_BTN_HOME] && !wii_dis)
	{
	    //must be held for 3 seconds
		if(!wii_togg_home)
		{
			home_press_begin = ros::Time::now();

			wii_togg_home = true;
		}
	    
	    
		if(wii_togg_home && state.buttons[state.MSG_BTN_HOME] && (ros::Time::now() - home_press_begin) > ros::Duration(3))
		{
			stop_robot();
			system("rosnode kill wiimote");
			wii_togg_home = false;
			wii_dis = true;
			wiimote_init = true;
			say("wiimote disconnected..  Please press the one and two buttons to reconnect");
			mode = "standby";
			display_mode();
			

		}
	}

	
	
	//wiimote behavior in wiimote mode
	if(mode == "wiimote")
	{
	
	    //initalize twist
		wii_twist.angular.x = 0;
		wii_twist.angular.y = 0;
		wii_twist.angular.z = 0;
		wii_twist.linear.x = 0;
		wii_twist.linear.y = 0;
		wii_twist.linear.z = 0;
	
		//button 1 moves to standby
		if(state.buttons[state.MSG_BTN_1] && !wii_togg_one)
		{
			mode = "standby";
			display_mode();
			wii_togg_one = true;
		}
		else if(!state.buttons[state.MSG_BTN_1])
		{
			wii_togg_one = false;
		}
		//button 2 moves to autonomous
		if(state.buttons[state.MSG_BTN_2] && !wii_togg_two)
		{
			mode = "autonomous";
			display_mode();
			wii_togg_two = true;
		}
		else if(!state.buttons[state.MSG_BTN_2])
		{
			wii_togg_two = false;
		}
	
		//setup local variables
		bool nunchuk_conected = false;
		double turbo_x = 1;
		double turbo_y = 1;
	
		if( state.nunchuk_joystick_raw[0] != 0 ||
			state.nunchuk_joystick_raw[1] != 0 )
		{
			nunchuk_conected = true;
		}
	
		if(nunchuk_conected)
		{
		    //nunchuk boost handlers
			if(state.nunchuk_buttons[state.MSG_BTN_Z])
			{
				turbo_x = turbo_x + params.turbo_x;
				turbo_y = turbo_y + params.turbo_y;
			}
			if(state.nunchuk_buttons[state.MSG_BTN_C])
			{
				turbo_x = turbo_x + params.turbo_x;
				turbo_y = turbo_y + params.turbo_y;
			}
		    
		    //compute controlls
			wii_twist.angular.z = state.nunchuk_joystick_zeroed[0] * params.base_rot_speed * turbo_x;
			wii_twist.linear.y  = state.nunchuk_joystick_zeroed[1] * params.base_linear_speed * turbo_y;
		}
		else
		{
		    //there constants for the buttons are wrong
		    //b button boost
			if(state.buttons[5])
			{
				turbo_x = turbo_x + params.turbo_x;
				turbo_y = turbo_y + params.turbo_y;
			}
			
			//accelerometer handlers a button
			if(state.buttons[4])
			{
				wii_twist.angular.z = (state.linear_acceleration_raw.x - 125) / 30  * params.base_rot_speed * turbo_x;
				wii_twist.linear.y  = -(state.linear_acceleration_raw.y - 125) / 30  * params.base_linear_speed * turbo_y;
			}
			//dpad handlers
			else

			{ 
			    //buttons are named with the wiimote on the side
				if(state.buttons[state.MSG_BTN_LEFT])
				{
					wii_twist.linear.y = params.base_linear_speed * turbo_y * params.d_pad_percent/100;
				}
				else if(state.buttons[state.MSG_BTN_RIGHT])
				{
					wii_twist.linear.y = -params.base_linear_speed * turbo_y * params.d_pad_percent/100;
				}
				if(state.buttons[state.MSG_BTN_UP])
				{
					wii_twist.angular.z = params.base_rot_speed * turbo_x * params.d_pad_percent/100;
				}
				else if(state.buttons[state.MSG_BTN_DOWN])
				{
					wii_twist.angular.z = -params.base_rot_speed * turbo_x * params.d_pad_percent/100;
				}
			}
			
		}
		
    }
    
    //wiimote behavior in autonomous mode
    if(mode=="autonomous")
    {
        //button 2 moves back to standby
    	if(state.buttons[state.MSG_BTN_2] && !wii_togg_two)
		{
			mode = "standby";
			display_mode();
			wii_togg_two = true;
		}
		else if(!state.buttons[state.MSG_BTN_2])
		{
			wii_togg_two = false;
		}
		
		//button 1 moves to wiimote mode
		if(state.buttons[state.MSG_BTN_1] && !wii_togg_one)
		{
			mode = "wiimote";
			display_mode();
			wii_togg_one = true;
		}
		else if(!state.buttons[state.MSG_BTN_1])
		{
			wii_togg_one = false;
		}
		
		//right button skips waypoint
		if(state.buttons[state.MSG_BTN_UP] && !wii_togg_up)
		{
			//resets waypoints
			say("reseting waypoints");
			system("rosrun dynamic_reconfigure dynparam set /Position reset_waypoints true");
			wii_togg_up = true;
		}
		else if(!state.buttons[state.MSG_BTN_UP])
		{
			wii_togg_up = false;
		}
		
		//left button resets waypoints
		if(state.buttons[state.MSG_BTN_DOWN] && !wii_togg_down)
		{

			//skips waypoint
			say("skip-ing waypoint");
			system("rosrun dynamic_reconfigure dynparam set /Position skip_waypoint true");
			wii_togg_down = true;
		}
		else if(!state.buttons[state.MSG_BTN_DOWN])
		{
			wii_togg_down = false;
		}
		
		//up button switches autonomous mode
		if(state.buttons[state.MSG_BTN_LEFT] && !wii_togg_left)
		{
			if(autonmous_mode == "navigation")
			{
				autonmous_mode = "carrot";
				say("Auto-bots,,, roll out!");
				system("rosrun dynamic_reconfigure dynparam set /Position go_to_waypoints false");
			}
			else if(autonmous_mode == "autonomous")
			{

				autonmous_mode = "navigation";
				say("moving to navigation waypoints");
				string temp = "rosrun dynamic_reconfigure dynparam load /Position " ;
				temp += params.navigation_waypoints;
				system(temp.c_str() );
				
			}
			else if(autonmous_mode == "carrot")
			{
				autonmous_mode = "autonomous";
				say("moving to autonomous waypoints");
				string temp = "rosrun dynamic_reconfigure dynparam load /Position " ;
				temp += params.autonomous_waypoints;
				ROS_INFO("%s",temp.c_str());
				system(temp.c_str());	
				
			}
			wii_togg_left = true;
		}
		else if(!state.buttons[state.MSG_BTN_LEFT])
		{
			wii_togg_left = false;
		}
		
		//down button switches autonomous mode
		if(state.buttons[state.MSG_BTN_RIGHT] && !wii_togg_right)
		{
			if(autonmous_mode == "navigation")
			{
				autonmous_mode = "autonomous";
				say("moving to autonomous waypoints");
				string temp = "rosrun dynamic_reconfigure dynparam load /Position " ;
				temp += params.autonomous_waypoints;
				system(temp.c_str());

;
			}
			else if(autonmous_mode == "autonomous")
			{
				autonmous_mode = "carrot";
				say("Auto-bots,,, roll out!");
				system("rosrun dynamic_reconfigure dynparam set /Position go_to_waypoints false");
				
			}
			else if(autonmous_mode == "carrot")
			{
				autonmous_mode = "navigation";
				say("moving to navigation waypoints");
				string temp = "rosrun dynamic_reconfigure dynparam load /Position " ;
				temp += params.navigation_waypoints;
				system(temp.c_str() );
				
			}

			wii_togg_right = true;
		}
		else if(!state.buttons[state.MSG_BTN_RIGHT])
		{
			wii_togg_right = false;
		}
		
	    //a button pauses operation
		if(state.buttons[4] && !wii_togg_a)
		{
			//pauses
			say("pausing waypoint navigation");
			system("rosrun dynamic_reconfigure dynparam set /Position pause true");
			wii_togg_a = true;
		}
		else if(!state.buttons[4])
		{
			wii_togg_a = false;
		}
		
		
    }

}


/***********************************************************
* @fn navigation_callback(const geometry_msgs::Twist::ConstPtr& twist)
* @brief decides on forwarding navigation comands to the motors
* @pre takes in a ros message of a twist from navigation and 
*      needs mode variable to be initalized
* @post publishes a CV_32FC1 image using cv_bridge
* @param takes in a ros message of a raw or cv image 
***********************************************************/
void navigation_callback(const geometry_msgs::Twist twist)
{
	if(mode=="autonomous" || mode=="jaus")		
	{
		//initalize twist
		nav_twist.angular.x = 0;
		nav_twist.angular.y = 0;
	    nav_twist.angular.z = twist.angular.z;
		nav_twist.linear.x = 0;
		nav_twist.linear.y = twist.linear.y;
		nav_twist.linear.z = 0;	
	}
}

/***********************************************************
* @fn navigation_callback(const geometry_msgs::Twist::ConstPtr& twist)
* @brief decides on forwarding navigation comands to the motors
* @pre takes in a ros message of a twist from navigation and 
*      needs mode variable to be initalized
* @post publishes a CV_32FC1 image using cv_bridge
* @param takes in a ros message of a raw or cv image 
***********************************************************/
void estop_callback(const MST_Estop::Estop_State::ConstPtr& estop)
{

		if(estop->state == 1 && !wii_togg_a)
		{
			//pauses
			say("pausing waypoint navigation");
			system("rosrun dynamic_reconfigure dynparam set /Position pause true");
			wii_togg_a = true;
		}
		else if(!estop->state)
		{
			wii_togg_a = false;
		}

}

/***********************************************************
* @fn setparamsCallback(const sensor_msgs::ImageConstPtr& msg)
* @brief callback for the reconfigure gui
* @pre has to have the setup for the reconfigure gui
* @post changes the parameters
***********************************************************/
void setparamsCallback(MST_Control::Control_ParamsConfig &config, uint32_t level)
{
  //ROS_INFO("Reconfigure request : %i %i %f %f %f %f %f ",
           //config.robot_x, config.robot_y, config.edges_per, config.lines_per, config.flags_per, 
           //config.obst_per, config.grass_per);
  
  
  
  // set params
  params = config;
  
}

void display_mode()
{
	
	wiimote::LEDControl led;
	sound_play::SoundRequest sound;
	
	
	if (mode == "standby")
	{
	    ROS_INFO("Control: Standby Mode");
	    
	    if(!wiimote_init)
	    {
	    	say("Joe-Mega-Tron standing by");
	    }
	 
	 	int lights[] ={0,1,1,0};

		led.set_timed_switch_array_size(4);
		for(int i=0; i < 4 ;i++)
		{
			led.timed_switch_array[i].switch_mode = lights[i];
			
		}
	    
	 	/*
		//setup nightrider
		float sizes[] ={4,6,6,4};
		float arrays[4][6]={{2 ,9,2,0,-1,-1},
				            {-1,1,3,5, 3, 1},
				            {-1,3,3,1, 3, 3},
				            {-1,5,3,5,-1,-1}};
		
		led.set_timed_switch_array_size(4);
		for(int i=0; i < 4 ;i++)
		{
			led.timed_switch_array[i].switch_mode = led.timed_switch_array[i].REPEAT;
			led.timed_switch_array[i].num_cycles = -1;
			led.timed_switch_array[i].set_pulse_pattern_size(sizes[i]);
			for(int j=0;j<sizes[i];j++)
			{
				led.timed_switch_array[i].pulse_pattern[j]=arrays[i][j] * (.05);
			}
		}
		*/
		
	}
	
	if (mode == "wiimote")
	{
	    ROS_INFO("Control: Wiimote Mode");
	    
	    say("Joe-Mega-Tron at you command");
	    
	 
		//turn on light 1
		int lights[] ={1,0,0,0};

		led.set_timed_switch_array_size(4);
		for(int i=0; i < 4 ;i++)
		{
			led.timed_switch_array[i].switch_mode = lights[i];
			

		}
		
	}

	if (mode == "autonomous")
	{
	    ROS_INFO("Control: Autonomous Mode");
	    
	    say("Intializing autonomous navigation");
	 
		//turn on light 2
		int lights[] ={0,1,0,0};

		led.set_timed_switch_array_size(4);
		for(int i=0; i < 4 ;i++)
		{
			led.timed_switch_array[i].switch_mode = lights[i];
		}
		
	}
	
	if (mode == "jaus")
	{
	    ROS_INFO("Control: Wiimote Mode");
	    
	    say("Joe-Mega-Tron under jaus control");
	 
		//turn on light 3
		int lights[] ={0,0,1,0};

		led.set_timed_switch_array_size(4);
		for(int i=0; i < 4 ;i++)
		{
			led.timed_switch_array[i].switch_mode = lights[i];
		}
		
	}
	
	wiimote_led_pub.publish(led);
	
}

void say(std::string say)
{
	sound_play::SoundRequest sound;
	
	sound.sound = sound.SAY;
	sound.command = sound.PLAY_ONCE;
	sound.arg = say;
	
	sound_pub.publish(sound);
}


void play(std::string play)
{
	sound_play::SoundRequest sound;
	
	sound.sound = sound.PLAY_FILE;
	sound.command = sound.PLAY_ONCE;
	sound.arg = play;
	
	sound_pub.publish(sound);
}

void stop_robot()
{
	geometry_msgs::Twist stop_twist;

	stop_twist.angular.x = 0;
	stop_twist.angular.y = 0;
	stop_twist.angular.z = 0;
	stop_twist.linear.x = 0;
	stop_twist.linear.y = 0;
	stop_twist.linear.z = 0;
	
	motor_pub.publish(stop_twist);
}

/***********************************************************
* @fn main(int argc, char **argv)state.buttons[state.MSG_BTN_HOME]
* @brief starts the Controll node and publishes motor comands
***********************************************************/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Control");
    ros::NodeHandle n;
 
    std::string nav;
	MST_Estop::Control_State  control_state;
	
    bool stoped = true;
    wiimote_init = true; 
    robot_init =true;  
    wii_dis = true;
    mode = "standby";
    autonmous_mode = "carrot";
	system("rosrun dynamic_reconfigure dynparam set /Position go_to_waypoints false");
    wii_togg_one = false;
    wii_togg_two = false;
    wii_togg_up  = false;
    wii_togg_down = false;
    wii_togg_left = false;
    wii_togg_right = false;
    wii_togg_plus = false;
    wii_togg_minus = false;
    wii_togg_home = false;
    wii_togg_a = false;
	
    
    
   	//setup dynamic reconfigure gui
    dynamic_reconfigure::Server<MST_Control::Control_ParamsConfig> srv;
    dynamic_reconfigure::Server<MST_Control::Control_ParamsConfig>::CallbackType f;
    f = boost::bind(&setparamsCallback, _1, _2);
	srv.setCallback(f);
    

    
    //get topic name
    nav = n.resolveName("nav_twist");

	//check to see if user has defined an image to subscribe to 
    if (nav == "nav_twist") 
    {
		ROS_WARN("Control: navigation twist has not beeen remaped! Typical command-line usage:\n"
				 "\t$ ./Contestop_pubrol twist:=<twist topic> [transport]");
    }
    
	
	
	nav_sub = n.subscribe( nav ,100, navigation_callback);
	
	pos_sub = n.subscribe( "/target" ,100, pos_callback);

	wiimote_state_sub = n.subscribe("wiimote/state" ,100,wiimote_callback);

	estop_sub = n.subscribe("/EStop" ,100 , estop_callback);
	
	wiimote_led_pub = n.advertise<wiimote::LEDControl>("wiimote/leds" ,100);
	
	wiimote_rum_pub = n.advertise<wiimote::RumbleControl>("wiimote/rumble" ,100);

	
	motor_pub = n.advertise<geometry_msgs::Twist>("cmd_vel" ,100);
	
	sound_pub = n.advertise<sound_play::SoundRequest>("robotsound" ,100);
	
 	estop_pub = n.advertise<MST_Estop::Control_State>("controlstate" ,100);
	
	//set rate to 30 hz
    ros::Rate loop_rate(30);
    
    
    
    //run main loop
	while (ros::ok())
    {
		//check calbacks
		ros::spinOnce();
		
		control_state.mode = mode;	
		
		estop_pub.publish(control_state);	
		
		if(robot_init)
		{
		    ros::Duration(7).sleep();
		    
        	say("Hello World. My name is Joe-Mega-Tron. Please press the one and two buttons on the wiimote to connect");
        	
        	robot_init = false;
		}
		if(mode=="standby")
		{
			if(!stoped)
			{
				stop_robot();
				stoped = true;
			}
		}
		else if(mode == "wiimote")
		{
			motor_pub.publish(wii_twist);
			stoped = false;
		}
		else if(mode == "autonomous")
		{
			motor_pub.publish(nav_twist);
			stoped = false;
		}
		
		
		loop_rate.sleep();
    }
    
    return 0;
}

