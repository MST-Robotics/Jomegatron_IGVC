/*******************************************************************************
* @file control.cpp
* @author James Anderson <jra798>
* @version 1.1
* @date 1/21/12
* @brief controlls which output goes to motors 
******************************************************************************/
#include    "control.h"


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
            change_mode( standby );
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
void wiimote_callback(const wiimote::State::ConstPtr& state)
{
    //sets inital conection and resets if node restarted
    //when wiimote is killed and restarted it continuously
    // publishes its last message the checking of home handels this 
    if(wiimote_init && (!state->buttons[MSG_BTN_HOME] && wii_dis))
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
        
        change_mode(standby);
        
        wii_dis = false;
        wiimote_init = false;
    }
    
    
    //plus an minus sounds plus and minus ar mixed up with 
    //a and b in the the apis map
    
    //plus
    if(check_togg(state->buttons[MSG_BTN_PLUS], MSG_BTN_PLUS))
    {
        say(params.plus_mesage);
    }
    
    //minus
    if(check_togg(state->buttons[MSG_BTN_MINUS], MSG_BTN_MINUS))
    {
        play(params.minus_sound);
    }
    
    //shut down code on 
    if(state->buttons[MSG_BTN_HOME] && !wii_dis)
    {
        //must be held for 3 seconds
        if(!wii_togg[MSG_BTN_HOME])
        {
            home_press_begin = ros::Time::now();

            wii_togg[MSG_BTN_HOME] = true;
        }
        
        
        if(wii_togg[MSG_BTN_HOME] && state->buttons[MSG_BTN_HOME] && (ros::Time::now() - home_press_begin) > ros::Duration(3))
        {
            stop_robot();
            system("rosnode kill wiimote");
            wii_togg[MSG_BTN_HOME] = false;
            wii_dis = true;
            wiimote_init = true;
            say("wiimote disconnected..  Please press the one and two buttons to reconnect");
            change_mode(standby);
            
        }
    }

    
    //behavior in stanby or jaus mode
    if(mode_ == standby || mode_ == jaus)
    {
        //1 button
        if(check_togg(state->buttons[MSG_BTN_1], MSG_BTN_1))
        {
            change_mode(wiimote_mode);
        }
        
        //two button
        if(check_togg(state->buttons[MSG_BTN_2], MSG_BTN_2))
        {
            change_mode(autonomous);     
        }
        
    }
    
    //wiimote behavior in wiimote mode
    if(mode_ == wiimote_mode)
    {
    
        //initalize twist
        wii_twist.angular.x = 0;
        wii_twist.angular.y = 0;
        wii_twist.angular.z = 0;
        wii_twist.linear.x = 0;
        wii_twist.linear.y = 0;
        wii_twist.linear.z = 0;
    
        //button 1 moves to standby
        if(check_togg(state->buttons[MSG_BTN_1], MSG_BTN_1))
        {
            change_mode(standby);
        }

        //button 2 moves to autonomous
        if(check_togg(state->buttons[MSG_BTN_2], MSG_BTN_2))
        {
            change_mode(autonomous);
        }
    
        //setup local variables
        bool nunchuk_conected = false;
        double turbo_x = 1;
        double turbo_y = 1;
    
        if( state->nunchuk_joystick_raw[0] != 0 ||
            state->nunchuk_joystick_raw[1] != 0 )
        {
            nunchuk_conected = true;
        }
    
        if(nunchuk_conected)
        {
            //nunchuk boost handlers
            if(state->nunchuk_buttons[MSG_BTN_Z])
            {
                turbo_x += params.turbo_x;
                turbo_y += params.turbo_y;
            }
            if(state->nunchuk_buttons[MSG_BTN_C])
            {
                turbo_x += params.turbo_x;
                turbo_y += params.turbo_y;
            }
            
            //compute controlls
            wii_twist.angular.z = state->nunchuk_joystick_zeroed[0] * params.base_rot_speed * turbo_x;
            wii_twist.linear.x  = state->nunchuk_joystick_zeroed[1] * params.base_linear_speed * turbo_y;
        }
        else
        {
            //there constants for the buttons are wrong
            //b button boost
            if(state->buttons[MSG_BTN_B])
            {
                turbo_x += params.turbo_x;
                turbo_y += params.turbo_y;
            }
            
            //accelerometer handlers a button
            if(state->buttons[MSG_BTN_A])
            {
                wii_twist.angular.z = -(state->linear_acceleration_zeroed.x) / 10  * params.base_rot_speed * turbo_x;
                wii_twist.linear.x  = (state->linear_acceleration_zeroed.y) / 10  * params.base_linear_speed * turbo_y;
            }
            //dpad handlers
            else
            { 
                //buttons are named with the wiimote on the side
                if(state->buttons[MSG_BTN_LEFT])
                {
                    wii_twist.linear.x = params.base_linear_speed * turbo_y * params.d_pad_percent/100;
                }
                else if(state->buttons[MSG_BTN_RIGHT])
                {
                    wii_twist.linear.x = -params.base_linear_speed * turbo_y * params.d_pad_percent/100;
                }
                if(state->buttons[MSG_BTN_UP])
                {
                    wii_twist.angular.z = params.base_rot_speed * turbo_x * params.d_pad_percent/100;
                }
                else if(state->buttons[MSG_BTN_DOWN])
                {
                    wii_twist.angular.z = -params.base_rot_speed * turbo_x * params.d_pad_percent/100;
                }
            }
            
        }
        
    }
    
    //wiimote behavior in autonomous mode
    if(mode_==autonomous)
    {
        //button 2 moves back to standby
        if(check_togg(state->buttons[MSG_BTN_2], MSG_BTN_2))
        {
            change_mode(standby);
        }
        
        //button 1 moves to wiimote mode
        if(check_togg(state->buttons[MSG_BTN_1], MSG_BTN_1))
        {
            change_mode(wiimote_mode);
        }
        
        //right button skips waypoint
        if(check_togg(state->buttons[MSG_BTN_UP], MSG_BTN_UP))
        {
            //resets waypoints
            say("reseting waypoints");
            system("rosrun dynamic_reconfigure dynparam set /Position reset_waypoints true");
        }
        
        //left button resets waypoints
        if(check_togg(state->buttons[MSG_BTN_DOWN], MSG_BTN_DOWN))
        {

            //skips waypoint
            say("skip-ing waypoint");
            system("rosrun dynamic_reconfigure dynparam set /Position skip_waypoint true");

        }
        
        //up button switches autonomous mode
        if(check_togg(state->buttons[MSG_BTN_LEFT], MSG_BTN_LEFT))
        {
            if(autonmous_mode == navigation)
            {
                autonmous_mode = carrot;
                say("Auto-bots,,, roll out!");
                system("rosrun dynamic_reconfigure dynparam set /Position go_to_waypoints false");
            }
            else if(autonmous_mode == autonomous_waypoints)
            {

                autonmous_mode = navigation;
                say("moving to navigation waypoints");
                string temp = "rosrun dynamic_reconfigure dynparam load /Position " ;
                temp += params.navigation_waypoints;
                system(temp.c_str() );
                
            }
            else if(autonmous_mode == carrot)
            {
                autonmous_mode = autonomous_waypoints;
                say("moving to autonomous waypoints");
                string temp = "rosrun dynamic_reconfigure dynparam load /Position " ;
                temp += params.autonomous_waypoints;
                ROS_INFO("%s",temp.c_str());
                system(temp.c_str());   
                
            }

        }
        
        //down button switches autonomous mode
        if(check_togg(state->buttons[MSG_BTN_RIGHT], MSG_BTN_RIGHT))
        {
            if(autonmous_mode == navigation)
            {
                autonmous_mode = autonomous_waypoints;
                say("moving to autonomous waypoints");
                string temp = "rosrun dynamic_reconfigure dynparam load /Position " ;
                temp += params.autonomous_waypoints;
                system(temp.c_str());
            }
            else if(autonmous_mode == autonomous_waypoints)
            {
                autonmous_mode = carrot;
                say("Auto-bots,,, roll out!");
                system("rosrun dynamic_reconfigure dynparam set /Position go_to_waypoints false");
                
            }
            else if(autonmous_mode == carrot)
            {
                autonmous_mode = navigation;
                say("moving to navigation waypoints");
                string temp = "rosrun dynamic_reconfigure dynparam load /Position " ;
                temp += params.navigation_waypoints;
                system(temp.c_str() );
                
            }

        }
        
        //a button pauses operation
        if(check_togg(state->buttons[MSG_BTN_A], MSG_BTN_A))
        {
            //pauses
            say("pausing waypoint navigation");
            system("rosrun dynamic_reconfigure dynparam set /Position pause true");
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
    if(mode_==autonomous || mode_==jaus)      
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

/***********************************************************
* @fn jaus_callback(const MST_JAUS::JAUS_out::ConstPtr& msg)
* @brief receives instructions from JAUS
* @pre takes in a message containing instructions from JAUS
* @post gives control to JAUS if requested, sets JAUS mode,
*       and passes on waypoints and speed
* @param takes in a message from MST_JAUS 
***********************************************************/
void jaus_callback(const MST_JAUS::JAUS_out::ConstPtr& msg)
{
    if(msg->request_control)
    {
        mode_ = jaus;
    }
    
    if(mode_ == jaus) //JAUS COP has control
    {
        if(msg->request_resume)
            jaus_mode = jaus_resume;
        else if(msg->request_standby)
            jaus_mode = jaus_standby;
        else if(msg->request_shutdown)
        {
            jaus_mode = jaus_shutdown;
            mode_ = standby; //BAD, just handing control to w/e for now
        }
        
        if(jaus_mode = jaus_resume)
            jaus_execute = msg->execute_waypoints;
        
        //Create/update list of waypoints
        for(int i = 0; i < msg->waypoint_id.size(); i++)
        {
            bool updated = false;
            
            for(int j = 0; j < jaus_waypoints.size(); j++)
            {
                if(jaus_waypoints[j]->ID = msg->waypoint_id[i])
                {
                    //Update
                    jaus_waypoints[j]->nextID = msg->waypoint_previous_id[i];
                    jaus_waypoints[j]->previousID = msg->waypoint_next_id[i];
                    jaus_waypoints[j]->x = msg->waypoint_pose_x[i];
                    jaus_waypoints[j]->y = msg->waypoint_pose_y[i];
                    jaus_waypoints[j]->yaw = msg->pose_yaw[i];
                    
                    updated = true;
                }
            }
            
            if(!updated)
            {
                jaus_waypoints.push_back(new jaus_waypoint);
                jaus_waypoints.back()->ID = msg->waypoint_id[i];
                jaus_waypoints.back()->nextID = msg->waypoint_previous_id[i];
                jaus_waypoints.back()->previousID = msg->waypoint_next_id[i];
                jaus_waypoints.back()->x = msg->waypoint_pose_x[i];
                jaus_waypoints.back()->y = msg->waypoint_pose_y[i];
                jaus_waypoints.back()->yaw = msg->pose_yaw[i];
            }
        }
        
        //Sort waypoints
        int current = -1;
        for(int i = 0; i < jaus_waypoints.size(); i++) //find first
        {
            if(jaus_waypoints[i]->previousID == 0) {
                current = i;
                break;
            }
        }
        for(int i = 0; i < jaus_waypoints.size(); i++)
        {
            jaus_waypoints[current]->priority = i+1;
            
            //find next
            for(int j = 0; j < jaus_waypoints.size(); i++)
            {
                if(jaus_waypoints[current]->nextID == jaus_waypoints[j]->ID)
                {
                    current = j;
                    break;
                }
            }
        }
        
        //Send to ROS param server
        char buf[27];
        for(int i=0; i<10; i++)
        {
            sprintf(buf, "/Position/way_%d_priority", i+1);
            if(jaus_waypoints.size() > i)
            {
                ros::param::set(buf, jaus_waypoints[8]->priority);
                sprintf(buf, "/Position/way_%d_latitude", i+1);
                ros::param::set(buf, jaus_waypoints[8]->y);//latitude);
                sprintf(buf, "/Position/way_%d_logitude", i+1);
                ros::param::set(buf, jaus_waypoints[8]->x);//longitude);
            }
            else
                ros::param::set(buf, 0);
        }
        
        //Set waypoint list to reply message
        jaus_msg.waypoint_id.clear();
        jaus_msg.waypoint_previous_id.clear();
        jaus_msg.waypoint_next_id.clear();
        jaus_msg.waypoint_x.clear();
        jaus_msg.waypoint_y.clear();
        for(int i = 0; i < jaus_waypoints.size(); i++)
        {
            jaus_msg.waypoint_id.push_back(jaus_waypoints[i]->ID);
            jaus_msg.waypoint_previous_id.push_back(jaus_waypoints[i]->previousID);
            jaus_msg.waypoint_next_id.push_back(jaus_waypoints[i]->nextID);
            jaus_msg.waypoint_x.push_back(jaus_waypoints[i]->x);
            jaus_msg.waypoint_y.push_back(jaus_waypoints[i]->y);
        }
        jaus_msg.waypoint_list_valid = true;
        
        //Find active waypoint ID
        int active = 0;
        for(int i=10; i>0; i--)
        {
            sprintf(buf, "/Position/way_%d_priority", i);
            int temp = 0;
            ros::param::get(buf, temp);
            if(temp != 0 && temp < active)
                active = temp;
        }
        jaus_msg.active_waypoint_id = jaus_waypoints[current]->ID;
    }
}

/***********************************************************
* @fn midg_callback(const mst_midg::IMU::ConstPtr& msg)
* @brief receives GPS info to send to JAUS
* @pre takes in a message from midg
* @post sets the appropriate jaus_msg fields
* @param takes in an IMU message from mst_midg
***********************************************************/
void midg_callback(const mst_midg::IMU::ConstPtr& imu)
{
    jaus_msg.position_valid = imu->position_valid;
    jaus_msg.gps_time = imu->gps_time;
    jaus_msg.latitude = imu->latitude;
    jaus_msg.longitude = imu->longitude;
    jaus_msg.altitude = imu->altitude;
    jaus_msg.position_accuracy = imu->position_accuracy;

    jaus_msg.heading_valid = imu->heading_valid;
    jaus_msg.heading = imu->heading;
    
    jaus_msg.speed_valid = imu->speed_valid;
    jaus_msg.speed = imu->speed;
    
    jaus_msg.angular_rate_valid = imu->angular_rate_valid;
    jaus_msg.angular_rate = imu->angular_rate;
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
    wiimote_init = true; 
    robot_init = true;  
    wii_dis = true;
    mode_ = standby;
    autonmous_mode = carrot;
    system("rosrun dynamic_reconfigure dynparam set /Position go_to_waypoints false");
    
    //initalize toggles 
    for (int i = 0; i < 30 ; i++)
    {
        wii_togg[i] = false;
    }
    
    
    
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

    wiimote_state_sub = n.subscribe("wiimote/state" ,100,wiimote_callback);
    
    jaus_sub = n.subscribe("/jaus_out" ,100, jaus_callback);

    estop_sub = n.subscribe("/EStop" ,100 , estop_callback);
    
    wiimote_led_pub = n.advertise<wiimote::LEDControl>("wiimote/leds" ,100);
    
    wiimote_rum_pub = n.advertise<wiimote::RumbleControl>("wiimote/rumble" ,100);

    
    //create publications
    motor_pub = n.advertise<geometry_msgs::Twist>("cmd_vel" ,100);
    
    sound_pub = n.advertise<sound_play::SoundRequest>("robotsound" ,100);
    
    jaus_pub = n.advertise<MST_JAUS::JAUS_in>("/jaus_in" ,100);

    
    //set rate to 30 hz
    ros::Rate loop_rate(30);
    
    
    
    //run main loop
    while (ros::ok())
    {
        //check calbacks
        ros::spinOnce();
        
        

        
        if(robot_init)
        {
            ros::Duration(7).sleep();
            
            say("Hello World. My name is Joe-Mega-Tron. Please press the one and two buttons on the wiimote to connect");
            
            robot_init = false;
        }
        if(mode_==standby)
        {
            if(!stopped)
            {
                stop_robot();
                stopped = true;
            }
        }
        else if(mode_ == wiimote_mode)
        {
            motor_pub.publish(wii_twist);
            stopped = false;
        }
        else if(mode_ == autonomous)
        {
            motor_pub.publish(nav_twist);
            stopped = false;
        }
        else if(mode_ == jaus)
        {
            if(jaus_mode == jaus_resume)
            {
                if(jaus_execute)
                    motor_pub.publish(nav_twist);
                jaus_pub.publish(jaus_msg);
                stopped = false;
                
                //reset jaus_msg until new data from midg
                jaus_msg.position_valid = false;
                jaus_msg.heading_valid = false;
                jaus_msg.speed_valid = false;
                jaus_msg.angular_rate_valid = false;
                jaus_msg.waypoint_list_valid = false;
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
    }
    
    return 0;
}

//checks the button toggle and switches states
//will return true on rising edge of press
bool check_togg(bool button_state, int button_position)
{
    bool togg = false;
    
    if(button_state && !wii_togg[button_position])
    {
        wii_togg[button_position] = true;
        
        togg = true;
    }
    else if (!button_state)
    {
        wii_togg[button_position] = false;
    }
    
    return togg;
}


void change_mode(Mode new_mode)
{
    
    wiimote::LEDControl led;
    sound_play::SoundRequest sound;
    
    mode_ = new_mode;
    
    if (mode_ == standby)
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
        //                    o f o  f o f
        float arrays[4][6]={{ 2,9,1,-1,0,0},
                            {-1,1,3, 5,3,1},
                            {-1,3,3, 1,3,3},
                            {-1,5,3, 5,0,0}};
        
        led.set_timed_switch_array_size(4);
        for(int i=0; i < 4 ;i++)
        {
            led.timed_switch_array[i].switch_mode = led.timed_switch_array[i].REPEAT;
            led.timed_switch_array[i].num_cycles = led.timed_switch_array[i].FOREVER;
            led.timed_switch_array[i].set_pulse_pattern_size(sizes[i]);
            for(int j=0;j<sizes[i];j++)
            {
                led.timed_switch_array[i].pulse_pattern[j] = arrays[i][j] * (.1);
            }
        }
        */
        
    }
    
    if (mode_ == wiimote_mode)
    {
        ROS_INFO("Control: Wiimote Mode");
        
        say("Joe-Mega-Tron at your command");


        //turn on light 1
        int lights[] ={1,0,0,0};

        led.set_timed_switch_array_size(4);
        for(int i=0; i < 4 ;i++)
        {
            led.timed_switch_array[i].switch_mode = lights[i];
            
        }
        
    }

    if (mode_ == autonomous)
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
    
    if (mode_ == jaus)
    {
        ROS_INFO("Control: JAUS Mode");
        
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

