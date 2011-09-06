/*******************************************************************************
* @file estop.cpp
* @author Mike Chrisco <mcccnb>
* @version 1.0
* @brief publishes ESTOP state received from external ESTOP board over serial
* port CTS. Receives robot status and sends to ESTOP board over serial port RTS
******************************************************************************/


/***********************************************************
* ROS specific includes
***********************************************************/
#include <ros/ros.h>

/***********************************************************
* Message includes
***********************************************************/
#include <MST_Estop/Control_State.h>
#include <MST_Estop/Estop_State.h>

/***********************************************************
* Other includes
***********************************************************/
#include <iostream>
#include <fstream>
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>

/***********************************************************
* Global variables
***********************************************************/


ros::Subscriber                 control_sub;

ros::Publisher                  estop_pub;


MST_Estop::Estop_State    estop_state;

/***********************************************************
* Function prototypes
***********************************************************/

/***********************************************************
* Namespace Changes
***********************************************************/
using namespace std;


/***********************************************************
* Defines
***********************************************************/
#define ESTOP_COM_PORT "/dev/ttyUSB3"

int com_port = 0;

/***********************************************************
* @fn edgesCallback(const sensor_msgs::ImageConstPtr& msg)
* @brief adds the edges image to map using a weight 
* @pre takes in a ros message of a raw or cv image
* @post image added to the map
* @param takes in a ros message of a raw or cv image 
***********************************************************/
void updateState_cb( const MST_Estop::Control_State::ConstPtr& status)
{
	int rtscmd = TIOCM_RTS;
	//ROS_INFO("Position: gps message receved");
	if(status->mode == "autonomous")
	{
		ioctl(com_port, TIOCMBIC, &rtscmd); // clear RTS pin(flashing)
	}
	else
	{
		ioctl(com_port, TIOCMBIS, &rtscmd); // set the RTS pin.
	}

}

/***********************************************************

* @fn main(int argc, char **argv)
* @brief starts the Pot_Nav node and publishises twist when 
* it gets a new image asuming 30 hz
***********************************************************/
int main(int argc, char **argv)
{
	int status;

	ros::init(argc, argv, "Estop");
	ros::NodeHandle n;

	//create subsctiptions
    control_sub = n.subscribe("/controlstate", 20, updateState_cb );

    //create publishers
    estop_pub = n.advertise<MST_Estop::Estop_State>( "/EStop" , 5 );
    
    //set rate to 30 hz
    ros::Rate loop_rate(30);
    
	com_port = open(ESTOP_COM_PORT,O_RDONLY);	
	if(read(com_port,NULL,0) < 0)
	{
		ROS_ERROR("Not a valid port");
		return -1;
	}

	ROS_INFO("Opened ESTOP Port");

    //run main loop
	while (ros::ok())
    {
		//check calbacks
		ros::spinOnce();
		//publish stuff
		ioctl(com_port, TIOCMGET,&status);
		if(status & TIOCM_CTS) //ESTOP SET
		{
			estop_state.state = 1;
			ROS_INFO("ESTOPPED!");
		}
		else
		{
			estop_state.state = 0;
		}
		estop_pub.publish(estop_state);
		loop_rate.sleep();
    }
    
    return 0;
}

