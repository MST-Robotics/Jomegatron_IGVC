#ifndef JAUS_CONTROLLER_H

// ROS Includes
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "mst_midg/IMU.h"

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
	void MidgCallback( const mst_midg::IMU::ConstPtr& msg );
	//~ void ControlCallback( MST_Control::/* TODO */::ConstPtr& msg );
	//~ void PositionCallback( MST_Position::/* TODO */::ConstPtr& msg );
	void MotorsCallback( const geometry_msgs::Twist::ConstPtr& msg );
};

#endif
