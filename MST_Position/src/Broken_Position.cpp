/*******************************************************************************
* @file Position.cpp
* @author James Anderson <jra798>
* @version 1.0
* @brief publishes transform tree using gps and headings taken in from garmin
* and Midg and provides a service to transform gps coardinates into the world
* frame 
******************************************************************************/


/***********************************************************
* ROS specific includes
***********************************************************/
#include <ros/ros.h>

/***********************************************************
* Message includes
***********************************************************/
#include <geometry_msgs/Twist.h>
#include <Midg/IMU.h>
#include <sensor_msgs/NavSatFix.h>
#include <MST_Position/Target_Heading.h>

/***********************************************************
* Other includes
***********************************************************/
#include <iostream>
#include <fstream>
#include <vector>
#include <dynamic_reconfigure/server.h>
#include <MST_Position/Position_ParamsConfig.h>
#include <tf/transform_broadcaster.h>


/***********************************************************
* Global variables
***********************************************************/


ros::Subscriber                 midg_sub;
ros::Subscriber                 garmin_sub;

ros::Publisher                  target_pub;

ros::ServiceServer              gps_to_pose;

bool                            map_changed = 0;
bool                            first_callback = 1;

bool                            gps_fix;

double                          current_lat;
double                          current_lon;
double                          current_head;

double                          inital_lat;
double                          inital_lon;

double                          way_lat[10];
double                          way_lon[10];
int                             way_priority[10];
bool                            waypoint_complete[10];

bool                            skip_waypoint;
bool                            stoped;

int                             current_priority;

double chosen_lat;
double chosen_lon;

double chosen_dist = -1;
double chosen_brng = 0;
int chosen_index = 99;

MST_Position::Target_Heading    target_heading;

MST_Position::Position_ParamsConfig    params;

/***********************************************************
* Function prototypes
***********************************************************/
void send_target(bool);
void reset_waypoints();
void read_waypoints();
double find_dist(double, double , double ,double );
double find_heading(double, double , double ,double );

/***********************************************************
* Namespace Changes
***********************************************************/
using namespace std;


/***********************************************************
* Defines
***********************************************************/

#define pi 3.14159265

/***********************************************************
* Message Callbacks
***********************************************************/



/***********************************************************
* @fn edgesCallback(const sensor_msgs::ImageConstPtr& msg)
* @brief adds the edges image to map using a weight 
* @pre takes in a ros message of a raw or cv image
* @post image added to the map
* @param takes in a ros message of a raw or cv image 
***********************************************************/
void midgCallback(const  Midg::IMU::ConstPtr& imu)
{
	ROS_INFO("Position: IMU message receved");
	
	//if using midg
	if(!params.use_gpsd && !params.use_dummy)
	{

		if(imu->position_valid /*&& imu->heading_valid*/)
		{
		 	gps_fix = true;
		 	current_lat = imu->latitude / 180 * pi;
		 	current_lon = imu->longitude / 180 * pi;
		 	
		}
		
		current_head = imu->heading;
		
	}
	

}

/***********************************************************
* @fn edgesCallback(const sensor_msgs::ImageConstPtr& msg)
* @brief adds the edges image to map using a weight 
* @pre takes in a ros message of a raw or cv image
* @post image added to the map
* @param takes in a ros message of a raw or cv image 
***********************************************************/
void gpsCallback( const sensor_msgs::NavSatFix::ConstPtr& imu)
{
	ROS_INFO("Position: gps message receved");
	if(params.use_gpsd && !params.use_dummy)
	{

		
	}

}

/***********************************************************
* @fn setparamsCallback(const sensor_msgs::ImageConstPtr& msg)
* @brief callback for the reconfigure gui
* @pre has to have the setup for the reconfigure gui
* @post changes the parameters
***********************************************************/
void setparamsCallback(MST_Position::Position_ParamsConfig &config, uint32_t level)
{
  
	if(config.skip_waypoint)
	{
	config.skip_waypoint = false;
	skip_waypoint = true;
	}
  
	//if using dummy
	if(config.use_dummy)
	{

	 	gps_fix = true;
	 	current_lat = config.dummy_latitude / 180 * pi;
	 	current_lon = config.dummy_longitude / 180 * pi;
	 	current_head = config.dummy_heading;	
	
	}
	
	if(config.reset_waypoints)
	{
		config.reset_waypoints = false;
		reset_waypoints();
	}
	
	if(config.pause)
	{
		if (stoped )
		{
			stoped = false;
		}
		else
		{
			stoped = true;
		}
		config.pause = false;
	}
  
	// set params
	params = config;
	
	
	read_waypoints();
  
}

/***********************************************************
* Private Functions
***********************************************************/
/***********************************************************
* @fn send_target(bool skip)
* @brief finds the best target to go to
* @pre takes in a bool to skip the current target
* @post publishes a message to target_heading
***********************************************************/
void send_target(bool skip)
{
	vector<int>  potential_waypoints;
	//gps_fix = false;	

	for(int i = 0;i < 10 ; i++)
	{
		
		if(way_priority[i] == current_priority && !waypoint_complete[i])
		{
			potential_waypoints.push_back(i);
		}
	}
	if(potential_waypoints.size() <= 0)
	{
		if(current_priority >= 10)			
		{
			target_heading.done = true;
			target_heading.target_heading = 0;
			target_heading.distance = 0;
			target_heading.stop_robot = true;
			target_heading.waypoint = 0;
			
			target_pub.publish(target_heading);
		}
		else
		{
			//increment priority and try again
			current_priority ++;
			send_target(false);
			
		}
	}
	else 
	{

		
		
		for(unsigned int j = 0 ; j < potential_waypoints.size(); j++)
		{
		
		    // find distance using haversine formula
			int index = potential_waypoints[j];
			double lat = way_lat[index]/ 180 * pi;
 			double lon = way_lon[index]/180 * pi;
			//earths radius times c
			double dist = find_dist(current_lat,current_lon,chosen_lat,chosen_lon);
			

			if(dist < chosen_dist || chosen_dist < 0)
			{
				chosen_dist = dist;
				chosen_index = index;
				chosen_lat = lat;
				chosen_lon = lon;
				//find the bearing 
        		chosen_brng = find_heading(current_lat,current_lon,chosen_lat,chosen_lon);
								
			}
			
		}
		
		if(chosen_dist <= params.waypoint_radius || skip)
		{
			waypoint_complete[chosen_index] = true;
			skip_waypoint = false;
			send_target(false);	
		}
		else
		{
			target_heading.distance = find_dist(current_lat,current_lon,chosen_lat,chosen_lon);
			target_heading.waypoint = chosen_index + 1;
			//output in radians
			target_heading.target_heading = chosen_brng - current_head ; 
			target_heading.done = false ;
			target_heading.stop_robot = false ;
	
			target_pub.publish(target_heading);
		}
	}
	
}

/***********************************************************
* @fn reset_waypoints()
* @brief resets waypoints list
* @post priority and compleat waypoints reset
***********************************************************/
void reset_waypoints()
{
	read_waypoints();
    for(int i = 0 ; i < 10 ; i++)
    {
    	waypoint_complete[i] = false;
    	current_priority = 1;
    }
}

/***********************************************************
* @fn double find_dist(double lat1, double lon1 , double lat2 ,double lon2)
* @brief computes distace between two gps points
* @pre takes in lat and lon for two points
* @post returns a double with the distace in meeters between points
***********************************************************/
double find_dist(double lat1, double lon1 , double lat2 ,double lon2)
{
	// find distance using haversine formula

       // magic forumala we have no idea how it works, if it works, or why it works.

	double R = 6371000;
	double delta_lat = lat2 - lat1;
	double delta_lon = lon2 - lon1;
	double a = sin(delta_lat/2) * sin(delta_lat/2) + cos(lat1) * cos(lat2) * sin(delta_lon/2) * sin(delta_lon/2);
	double c =  2 * atan2(sqrt(a) ,sqrt(1 - a));
	
	//earths radius times c
	return  R * c ;
}

/***********************************************************
* @fn double find_heading(double lat1, double lon1 , double lat2 ,double lon2)
* @brief computes heading between two gps points
* @pre takes in lat and lon for two points
* @post returns a double with the heading in radians
***********************************************************/
double find_heading(double lat1, double lon1 , double lat2 ,double lon2)
{
	// find bearing between two gps points

	double delta_lon = lon2 - lon1;
	double heading;

        // magic forumla #2 we have no idea how it works, if it works or why it works

	if(delta_lon != 0)
	{	
		//find the bearing 
	     double x = sin(delta_lon) * cos(lat2);

		double y = cos(lat1) * sin(lat2) -
					sin(lat1) * cos(lat1) * cos(delta_lon);
		double bearing = atan2(x,y);
	
		heading = -bearing + ( pi/2 ); 

		ROS_INFO("Heading : %f", heading);
	}
	else
	{
		heading = 0 ;
	}
	
	return heading;

}

/***********************************************************
* @fn read_waypoints()
* @brief reads gps waypoinds from dynamic reconfigure
* @pre has to have global arrays to place params in
* @todo this should be replaced with reading a file
* @todo arrays should be a vector of structs
***********************************************************/
void read_waypoints()
{
	//read in waypoint 1
	way_lat[0] = params.way_1_latitude;
	way_lon[0] = params.way_1_longitude;
    way_priority[0] = params.way_1_priority;
    
    //read in waypoint 2
	way_lat[1] = params.way_2_latitude;
	way_lon[1] = params.way_2_longitude;
    way_priority[1] = params.way_2_priority;
    
    //read in waypoint 3
	way_lat[2] = params.way_3_latitude;
	way_lon[2] = params.way_3_longitude;
    way_priority[2] = params.way_3_priority;
    
    //read in waypoint 4
	way_lat[3] = params.way_4_latitude;
	way_lon[3] = params.way_4_longitude;
    way_priority[3] = params.way_4_priority;
    
    //read in waypoint 5
	way_lat[4] = params.way_5_latitude;
	way_lon[4] = params.way_5_longitude;
    way_priority[4] = params.way_5_priority;
    
    //read in waypoint 6
	way_lat[5] = params.way_6_latitude;
	way_lon[5] = params.way_6_longitude;
    way_priority[5] = params.way_6_priority;
    
    //read in waypoint 7
	way_lat[6] = params.way_7_latitude;
	way_lon[6] = params.way_7_longitude;
    way_priority[6] = params.way_7_priority;
    
    //read in waypoint 8
	way_lat[7] = params.way_8_latitude;
	way_lon[7] = params.way_8_longitude;
    way_priority[7] = params.way_8_priority;
    
    //read in waypoint 9
	way_lat[8] = params.way_9_latitude;
	way_lon[8] = params.way_9_longitude;
    way_priority[8] = params.way_9_priority;
    
    //read in waypoint 10
	way_lat[9] = params.way_10_latitude;
	way_lon[9] = params.way_10_longitude;
    way_priority[9] = params.way_10_priority;
    
    
}

void find_target()
{
	
}

/***********************************************************
* @fn main(int argc, char **argv)
* @brief starts the Pot_Nav node and publishises twist when 
* it gets a new image asuming 30 hz
***********************************************************/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Position");
    ros::NodeHandle n;
    
    bool inital_gps=true;
    gps_fix = false;
    skip_waypoint = false;
	int target_waypoint;

    //setup dynamic reconfigure
	dynamic_reconfigure::Server<MST_Position::Position_ParamsConfig> srv;
    dynamic_reconfigure::Server<MST_Position::Position_ParamsConfig>::CallbackType f;
    f = boost::bind(&setparamsCallback, _1, _2);
	srv.setCallback(f);

	//create subsctiptions
    midg_sub = n.subscribe( n.resolveName("midg") , 20, midgCallback );
    
    garmin_sub = n.subscribe( "fix" , 20, gpsCallback );

    //create publishers
    target_pub = n.advertise<MST_Position::Target_Heading>( "target" , 5 );
   
    reset_waypoints();
    
    //set rate to 30 hz
    ros::Rate loop_rate(30);
    
    //run main loop
	while (ros::ok())
    {
		//check calbacks
		ros::spinOnce();
		
		if(gps_fix)
		{
			//send the target to navigation
			if (inital_gps)
			{
				target_waypoint = find_target();
				inital_gps = false;
			}						
				
			target_heading = compute_msg(target_waypoint);

			if(target_heading.distace <= params.waypoint_radius || skip)
			{
				target_waypoint = find_target();
				target_heading = compute_msg(target_waypoint);
			}
			
		}
		
		loop_rate.sleep();
    }
    
    return 0;
}

