#ifndef JAUS_CONTROLLER_H

// ROS Includes
#include "ros/ros.h"
#include "MST_JAUS/JAUS_in.h"
#include "MST_JAUS/JAUS_out.h"

// JAUS++ includes
#include <jaus/mobility/sensors/globalposesensor.h>
#include <jaus/mobility/sensors/localposesensor.h>
#include <jaus/mobility/sensors/velocitystatesensor.h>
#include <jaus/mobility/drivers/localwaypointlistdriver.h>
#include <jaus/core/transport/judp.h>
#include <jaus/core/time.h>
#include <jaus/core/component.h>

#include "JAUS_Constants.h"

class JAUS_Controller
{
private:
    bool                fault;
	
    /*-----------------------------------
	ROS variables
	-----------------------------------*/
    ros::Subscriber     s_State;
	ros::Publisher  	p_Control;
    
	
	/*-----------------------------------
	JAUS++ variables
	-----------------------------------*/
	//~ JAUS::UShort 		subsystemID;
	//~ JAUS::Byte			nodeID;
	//~ JAUS::Byte			componentID;
	
	JAUS::Component		component;
	
	JAUS::GlobalPoseSensor *
						globalPoseSensor;
	JAUS::LocalPoseSensor *
						localPoseSensor;
	JAUS::VelocityStateSensor *
						velocityStateSensor;
	JAUS::LocalWaypointListDriver *
						localWaypointListDriver;
	JAUS::JUDP * 	transportService;
	
	JAUS::GlobalPose 	globalPose;
	JAUS::VelocityState	velocityState;
    
    void initialize_services();
    void initialize_subs_and_pubs(ros::NodeHandle n);
    
    class ControlCallback : public JAUS::Discovery::Callback {
    public:
        ControlCallback(JAUS_Controller* c): parent(c) {}
        ~ControlCallback() {}
        virtual void ProcessMessage(const JAUS::Message* message);
    private:
        JAUS_Controller* parent;
    };
    ControlCallback* controlCallback;

public:
	JAUS_Controller( ros::NodeHandle n );
	~JAUS_Controller();
    bool run();
    /*-----------------------------------
	ROS methods
	-----------------------------------*/
    void StateCallback(const MST_JAUS::JAUS_in::ConstPtr& msg);
};

#endif
