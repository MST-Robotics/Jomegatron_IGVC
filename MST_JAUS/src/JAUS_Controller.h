#ifndef JAUS_CONTROLLER_H

// ROS Includes
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "Midg/IMU.h"

// JAUS++ includes
#include <jaus/mobility/sensors/globalposesensor.h>
#include <jaus/mobility/sensors/localposesensor.h>
#include <jaus/mobility/sensors/velocitystatesensor.h>
#include <jaus/mobility/drivers/localwaypointlistdriver.h>
#include <jaus/core/transport/judp.h>
#include <jaus/core/time.h>
#include <jaus/core/component.h>

// Literal Constants
//~ #define SUBSYSTEM_ID        106
#define SUBSYSTEM_ID		1 
#define NODE_ID             1
#define COMPONENT_ID        1

//~ #define COP_IP              "192.168.1.42"
#define COP_IP				"10.0.200.103"
//~ #define COP_SUBSYSTEM_ID    42
#define COP_SUBSYSTEM_ID	103
#define COP_NODE_ID         1
#define COP_COMPONENT_ID    1

class JAUS_Controller
{
private:
    bool                fault;
	
    /*-----------------------------------
	ROS variables
	-----------------------------------*/
	ros::Subscriber 	s_Midg;
	ros::Subscriber 	s_Control;
	ros::Subscriber 	s_Position;
	ros::Subscriber 	s_Motors;
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

public:
	JAUS_Controller( ros::NodeHandle n );
	~JAUS_Controller();
    bool run();
    /*-----------------------------------
	ROS methods
	-----------------------------------*/
	void MidgCallback( const Midg::IMU::ConstPtr& msg );
	//~ void ControlCallback( MST_Control::/* TODO */::ConstPtr& msg );
	//~ void PositionCallback( MST_Position::/* TODO */::ConstPtr& msg );
	void MotorsCallback( const geometry_msgs::Twist::ConstPtr& msg );
};

#endif
