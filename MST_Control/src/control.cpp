/*******************************************************************************
* @file control.cpp
* @author Michael Lester
* @version 1.1
* @date 4/3/13
* @brief controlls which output goes to motors 
******************************************************************************/
#include    "control.h"
#include "geometry_msgs/Vector3.h"


/***********************************************************
* Message Callbacks
***********************************************************/



/***********************************************************
* @fn wiimote_callback(const wiimote::State::ConstPtr& state
* @brief handels input from the wiimote
* @pre takes in a wiimote state message
* @post computes wii_twist variable and changes mode
***********************************************************/
void pos_callback( const MST_Position::Target_Heading::ConstPtr& msg)
{
        if(msg->done && !done_togg)
        {
            play(params.done_sound);
            ros::Duration(6).sleep();
            //change_mode( standby );
            done_togg = true;
        }
        else if (!msg->done)
        {
            done_togg = false;
        }
        
        if(msg->waypoint != last_msg_waypoint)
        {
            say("moving to next waypoint");
            play(params.waypoint_sound);
            last_msg_waypoint = msg->waypoint;
        }
        //waypoint 

}

/***********************************************************
* Xbox CRAP
***********************************************************/
void xbox_callback(const sensor_msgs::Joy::ConstPtr& msg)
{                                    //xbox controller axes
    joy_rightstick_x = msg->axes[4];
    joy_rightstick_y = msg->axes[3];
    
    joy_leftstick_x  =msg->axes[1];  //each set of two represents our x as linear and y as angular, to change  joystick goup to Navigation::run()
    joy_leftstick_y  =msg->axes[0];
    
    joy_r_trigger    =msg->axes[5];
    joy_l_trigger    =msg->axes[2];
    /*                                 //xbox control buttons
    joy_button_A =msg->buttons[0];
    joy_button_B =msg->buttons[1];
    joy_button_X =msg->buttons[2];
    joy_button_Y =msg->buttons[3];
    joy_r_button =msg->buttons[5];
    joy_l_button =msg->buttons[4];
    joy_start_b  =msg->buttons[7];
    joy_back_b   =msg->buttons[6];
    joy_dpad_up  =msg->buttons[13];
    joy_dpad_dwn =msg->buttons[14];
    joy_dpad_l   =msg->buttons[11];
    joy_dpad_r   =msg->buttons[12];
    joy_light    =msg->buttons[8];
    joy_l_stick  =msg->buttons[9];
    joy_r_stick  =msg->buttons[10];
    */
    
    if(mode_ == xbox_mode)
    {
    
        //initalize twist 
        geometry_twist.angular.x = 0;
        geometry_twist.angular.y = 0;
        geometry_twist.angular.z = 0;
        geometry_twist.linear.x = 0;
        geometry_twist.linear.y = 0;
        geometry_twist.linear.z = 0;
        
        //Controller Behavior in controller mode
        //Assign buttons to change mode
        if(check_togg(msg->buttons[joy_button_Y], joy_button_Y))
        {
            change_mode(autonomous);
        }

        
        if(check_togg(msg->buttons[joy_button_B], joy_button_B))
        {
            change_mode(standby);
        }
            
            //Choose method of controlling the robot
                geometry_twist.angular.z = (joy_rightstick_y) / 10  * params.base_rot_speed;
                geometry_twist.linear.x  = (joy_leftstick_x) / 10  * params.base_linear_speed;
         
            
        
        
    }
    
    //Controller Behavior in autonomous mode
    //Assign buttons to change mode
    if(mode_ == autonomous)
    {
        
        if(check_togg(msg->buttons[joy_button_B], joy_button_B))                  
        {
            change_mode(standby);
        }
        
        
        if(check_togg(msg->buttons[joy_button_A], joy_button_B))
        {
            change_mode(xbox_mode);
        }
        
       
        
    }
    //Controller Behavior in standby mode
    if(mode_ == standby)
    {
        
        if(check_togg(msg->buttons[joy_button_Y], joy_button_Y))                  
        {
            change_mode(autonomous);
        }
        
        
        if(check_togg(msg->buttons[joy_button_A], joy_button_A))
        {
            change_mode(xbox_mode);
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
    if(mode_==autonomous)// || mode_==jaus)      
    {
        //initalize twist
        nav_twist.angular.x = 0;
        nav_twist.angular.y = 0;  
        nav_twist.angular.z = twist.angular.z;
        nav_twist.linear.x = twist.linear.x;
        nav_twist.linear.y = 0;
        nav_twist.linear.z = 0; 
    }
}

void estop_callback(const MST_Estop::Estop_State::ConstPtr& estop)
{

        if(estop->state == 1 && !estop_togg)
        {
            //pauses
            say("Joe-Mega-Tron E-Stopped");
            system("rosrun dynamic_reconfigure dynparam set /Position pause true");
            estop_togg = true;
        }
        else if(!estop->state)
        {
            estop_togg = false;
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
/***********************************************************
* @fn main(int argc, char **argv)state.buttons[state.MSG_BTN_HOME]
* @brief starts the Controll node and publishes motor comands
***********************************************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Control");
    ros::NodeHandle n;
 
    std::string nav;

    
    bool stopped = true;
    robot_init = true;  
    mode_ = standby;
    autonomous_mode = navigation;
    system("rosrun dynamic_reconfigure dynparam set /Position go_to_waypoints false");
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
    
    
    // create subscriptions
    nav_sub = n.subscribe( nav ,100, navigation_callback);
    
    pos_sub = n.subscribe( "/target" ,100, pos_callback);
    
    xbox_state_sub = n.subscribe("/joy", 100, xbox_callback);
    
    estop_sub = n.subscribe("/EStop" ,100 , estop_callback);
     
    motor_pub = n.advertise<geometry_msgs::Twist>("cmd_vel" ,100);
    
    sound_pub = n.advertise<sound_play::SoundRequest>("robotsound" ,100);

//set rate to 30 hz
    ros::Rate loop_rate(30);
    
    
    
    //run main loop
    while (ros::ok())
    {
        //check calbacks
        ros::spinOnce();
        
        
        
        
        if(robot_init)
        {
            ros::Duration(6).sleep();
            
            say("Hello World. My name is S and T Enterprise. Please press the EX box button to connect");
            ros::Duration(12).sleep();
            robot_init = false;
        }
        if(mode_ ==standby)
        {
            if(!stopped)
            {
                stop_robot();
                stopped = true;
            }
        }
        
        /*This is where you tell what your controller to publish*/
        
        else if(mode_ == xbox_mode)
        {
            motor_pub.publish(geometry_twist);
            stopped = false;
        }
        else if(mode_ == autonomous)
        {
            motor_pub.publish(nav_twist);
            stopped = false;
        }
        else
            {
                if(!stopped)
                {
                    stop_robot();
                    stopped = true;
                }
            }
        
        }
        
        loop_rate.sleep();
    
    
    return 0;
}

bool check_togg(bool button_state, int button_position)
{
    bool togg = false;
    
    if(button_state && !xtogg[button_position])
    {
        xtogg[button_position] = true;
        
        togg = true;
    }
    else if (!button_state)
    {
        xtogg[button_position] = false;
    }
    
    return togg;
}
void change_mode(Mode new_mode)
{
    
    //wiimote::LEDControl led;
    sound_play::SoundRequest sound;
    
    mode_ = new_mode;
    
    if (mode_ == standby)
    {
        ROS_INFO("Control: Standby Mode");
        say("Enterprise standing by");  
    }
    

    if (mode_ == autonomous)
    {
        ROS_INFO("Control: Autonomous Mode");
        
        say("Intializing autonomous navigation");
        
    }
    
    if (mode_ == xbox_mode)
    {
        ROS_INFO("Control: Xbox Mode");
        
        
        say("Controller Mode Initialized");
        say("Enterprise at your command");
    }
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
