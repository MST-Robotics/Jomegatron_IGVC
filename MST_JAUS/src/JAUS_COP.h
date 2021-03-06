#ifndef JAUS_COP_H

// ROS Includes
#include "ros/ros.h"

// JAUS++ includes
#include <jaus/mobility/sensors/globalposesensor.h>
#include <jaus/mobility/sensors/localposesensor.h>
#include <jaus/mobility/sensors/velocitystatesensor.h>
#include <jaus/mobility/drivers/localwaypointlistdriver.h>
#include <jaus/core/transport/judp.h>
#include <jaus/core/time.h>
#include <jaus/core/component.h>

// Other Includes
#include <ctime>
#include <pthread.h>
#include <string>
#include <iostream>
#include "JAUS_Constants.h"

class JAUS_COP
{
private:
    bool                fault;
    
    JAUS::Address       source;
    JAUS::Address       destination;
    bool                mst_jaus_discovered;
    bool                mst_jaus_controlled;
    JAUS::Byte          mst_jaus_status;
    std::time_t         statusTimer;
    
    pthread_t inputThread;
    bool shutdown;
	
    /*-----------------------------------
	ROS variables
	-----------------------------------*/
    
	
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
	JAUS_COP( ros::NodeHandle n );
	~JAUS_COP();
    bool run();
    bool discover(bool* run);
    bool requestControl();
    JAUS::Byte requestStatus();
    void printStatus(JAUS::Byte status) const;
    
    static void* getInput(void* ptr);
    
    /*-----------------------------------
	ROS methods
	-----------------------------------*/
	
};

#endif
