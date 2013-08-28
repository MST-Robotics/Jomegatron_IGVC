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
#include <mst_midg/IMU.h>
#include <sensor_msgs/NavSatFix.h>
#include <MST_Position/Target_Heading.h>
#include <nav_msgs/Odometry.h>

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
ros::Publisher                  odom_pub;
ros::Publisher                  goal_pub;

ros::ServiceServer              gps_to_pose;

bool                            map_changed = 0;
bool                            first_callback = 1;

bool                            gps_fix;

ros::Time                       current_time;
double                          current_lat;
double                          current_lon;
double                          current_head;

double                          inital_lat;
double                          inital_lon;
double                          inital_head;

double                          way_lat[10];
double                          way_lon[10];
int                             way_priority[10];
double                          way_limit[10];
bool                            waypoint_complete[10];

bool                            skip_waypoint;
bool                            stoped;

int                             current_priority;

bool inital_gps=true;

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
//void send_target(bool);
void reset_waypoints();
void read_waypoints();
double find_dist(double, double , double ,double );
double find_heading(double, double , double ,double );
int find_target();
MST_Position::Target_Heading compute_msg(int);
nav_msgs::Odometry odom_msg();
void odom_set_origin();


/***********************************************************
* Namespace Changes
***********************************************************/
using namespace std;


/***********************************************************
* Defines
***********************************************************/

#define pi M_PI

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
void midgCallback(const  mst_midg::IMU::ConstPtr& imu)
{
	ROS_INFO("Position: IMU message receved");
	
	//if using midg
	if(!params.use_gpsd && !params.use_dummy)
	{

		if(imu->position_valid)
		{
		 	gps_fix = true;
            current_time = ros::Time(imu->gps_time);
		 	//current_lat = imu->latitude; // 180 * pi;
		 	//current_lon = imu->longitude; // 180 * pi;
		}
        else
        {
            gps_fix =false;
        }
		
        if(imu->heading)
            current_head = imu->heading + params.heading_offset;
		
	}
	

}

/***********************************************************
* @fn edgesCallback(const sensor_msgs::ImageConstPtr& msg)
* @brief adds the edges image to map using a weight 
* @pre takes in a ros message of a raw or cv image
* @post image added to the map
* @param takes in a ros message of a raw or cv image 
***********************************************************/
void gpsCallback( const sensor_msgs::NavSatFix::ConstPtr& fix)
{
	ROS_INFO("Position: gps message received");
	if(params.use_gpsd && !params.use_dummy)
	{
     current_lon = fix->longitude;
     current_lat = fix->latitude;
  std::cout << "Current Latitude: " << current_lon << std::endl;
  std::cout << "Current Longitude: " << current_lat << std::endl;
		
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
        current_time = ros::Time::now();
	 	//current_lat = config.dummy_latitude; // 180 * pi;
	 	//current_lon = config.dummy_longitude; // 180 * pi;
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
/*
/***********************************************************
* @fn send_target(bool skip)
* @brief finds the best target to go to
* @pre takes in a bool to skip the current target
* @post publishes a message to target_heading
***********************************************************/
/*
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
*/
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
        //reset the waypoint complete values
    	waypoint_complete[i] = false;
    	
    }
    
    if(params.reverse_order)
	{
	    current_priority = 10;
	}
	else
	{
	    current_priority = 1;
	}
    
	inital_gps = true;
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

	//find the bearing 
    double x = sin(delta_lon) * cos(lat2);

	double y = cos(lat1) * sin(lat2) -
				sin(lat1) * cos(lat1) * cos(delta_lon);
	double bearing = atan2(x,y);

	heading = -bearing + ( pi/2 ); 

	//ROS_INFO("Heading : %f", heading);

	
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
	way_limit[0] = params.way_1_limit;
    
    //read in waypoint 2
	way_lat[1] = params.way_2_latitude;
	way_lon[1] = params.way_2_longitude;
    way_priority[1] = params.way_2_priority;
	way_limit[1] = params.way_2_limit;
    
    //read in waypoint 3
	way_lat[2] = params.way_3_latitude;
	way_lon[2] = params.way_3_longitude;
    way_priority[2] = params.way_3_priority;
	way_limit[2] = params.way_3_limit;
    
    //read in waypoint 4
	way_lat[3] = params.way_4_latitude;
	way_lon[3] = params.way_4_longitude;
    way_priority[3] = params.way_4_priority;
	way_limit[3] = params.way_4_limit;
    
    //read in waypoint 5
	way_lat[4] = params.way_5_latitude;
	way_lon[4] = params.way_5_longitude;
    way_priority[4] = params.way_5_priority;
	way_limit[4] = params.way_5_limit;
    
    //read in waypoint 6
	way_lat[5] = params.way_6_latitude;
	way_lon[5] = params.way_6_longitude;
    way_priority[5] = params.way_6_priority;
	way_limit[5] = params.way_5_limit;
    
    //read in waypoint 7
	way_lat[6] = params.way_7_latitude;
	way_lon[6] = params.way_7_longitude;
    way_priority[6] = params.way_7_priority;
	way_limit[6] = params.way_7_limit;
    
    //read in waypoint 8
	way_lat[7] = params.way_8_latitude;
	way_lon[7] = params.way_8_longitude;
    way_priority[7] = params.way_8_priority;
	way_limit[7] = params.way_7_limit;
    
    //read in waypoint 9
	way_lat[8] = params.way_9_latitude;
	way_lon[8] = params.way_9_longitude;
    way_priority[8] = params.way_9_priority;
	way_limit[8] = params.way_8_limit;
    
    //read in waypoint 10
	way_lat[9] = params.way_10_latitude;
	way_lon[9] = params.way_10_longitude;
    way_priority[9] = params.way_10_priority;
	way_limit[9] = params.way_10_limit;
    
    
}

int find_target()
{

	double lowest_dist = -1;
	int closest_target = -1;
	while (closest_target == -1 && ((current_priority <= 10 && !params.reverse_order) || (current_priority >= 1 && params.reverse_order)) )
	{
		for(int i = 0 ; i < 10 ; i++)
		{
			if(!waypoint_complete[i] && way_priority[i] !=0 && (((way_priority[i] <= current_priority) && !params.reverse_order) || ((way_priority[i] >= current_priority) && params.reverse_order)))
			{
				double lat = way_lat[i]/180 * pi ;
				double lon = way_lon[i]/180 * pi;
				ROS_INFO("Lat: %f Lon: %f",lat , lon);
				double dist = find_dist(current_lat,current_lon,lat,lon);

				if(dist <= lowest_dist || lowest_dist == -1)
				{
					lowest_dist = dist;
					closest_target = i;
				}
			
			}
		}
		if(closest_target == -1)
		{
		    if(params.reverse_order)
		    {   
		        current_priority--;
		    }
		    else
		    {
			    current_priority ++;
		}	}
	}
	

	return closest_target;	
	
}

MST_Position::Target_Heading compute_msg(int target)
{
	MST_Position::Target_Heading heading;
	double lat = way_lat[target] / 180 * pi ;
	double lon = way_lon[target] / 180 * pi; 
 	

	heading.target_heading = current_head + pi/2 - find_heading(current_lat,current_lon,lat,lon);
	heading.distance = find_dist(current_lat,current_lon,lat,lon);
	heading.waypoint = target;
	heading.done = false;
	heading.stop_robot = false;

	return heading;
}

// Programmer: Jason Gassel  Date: 3-5-12
// Descr: Generates Odometry message
nav_msgs::Odometry odom_msg()
{
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "midg_link";
    
    //Calculate local coord from lat/lon
    double dist = find_dist(inital_lat, inital_lon, current_lat, current_lon);
    double theta = find_heading(inital_lat, inital_lon, current_lat, current_lon);
    double dist_x = cos(theta) * dist;
    double dist_y = sin(theta) * dist;
    double heading = current_head - inital_head;
    
    //Fill in odom message
    odom.pose.pose.position.x = dist_x;
    odom.pose.pose.position.y = dist_y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(heading);
    double cov_x = 9999999, cov_y = 9999999, cov_z = 9999999;
    double cov_qx = 10, cov_qy = 10, cov_qz = 10;
    if(gps_fix)
    {
        cov_x = 10;
        cov_y = 10;
        cov_z = 10;
    }
    double temp[] = {cov_x, 0, 0, 0, 0, 0,
                     0, cov_y, 0, 0, 0, 0,
                     0, 0, cov_z, 0, 0, 0,
                     0, 0, 0, cov_qx, 0, 0,
                     0, 0, 0, 0, cov_qy, 0,
                     0, 0, 0, 0, 0, cov_qz};
    for(int i=0; i<36; ++i)
        odom.pose.covariance[i] = temp[i];
    double temp2[] = {9999999, 0, 0, 0, 0, 0,
                     0, 9999999, 0, 0, 0, 0,
                     0, 0, 9999999, 0, 0, 0,
                     0, 0, 0, 9999999, 0, 0,
                     0, 0, 0, 0, 9999999, 0,
                     0, 0, 0, 0, 0, 9999999};
    for(int i=0; i<36; ++i)
        odom.twist.covariance[i] = temp2[i];
    
    return odom;
}

void odom_set_origin()
{
    inital_lat = current_lat;
    inital_lon = current_lon;
    inital_head = current_head;
    
    return;
}

geometry_msgs::PoseStamped relative_waypoint(int target)
{
    geometry_msgs::PoseStamped waypoint;
    waypoint.header.stamp = current_time;
    waypoint.header.frame_id = "goal";
    
    double dist = find_dist(current_lat, current_lon, way_lat[target], way_lon[target]);
    double theta = find_heading(current_lat, current_lon, way_lat[target], way_lon[target]);
    waypoint.pose.position.x = cos(theta) * dist;
    waypoint.pose.position.y = sin(theta) * dist;
    waypoint.pose.position.z = 0;
    
    return waypoint;
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
    
    
    gps_fix = false;
    skip_waypoint = false;
	int target_waypoint = -1;

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
        
    odom_pub = n.advertise<nav_msgs::Odometry>( "vo", 5 );
    
    goal_pub = n.advertise<geometry_msgs::PoseStamped>( "move_base_simple/goal", 5 );
   
    reset_waypoints();
    
    
    //set rate to 30 hz
    ros::Rate loop_rate(30);
    
    //run main loop
	while (ros::ok())
    {
		//check calbacks
		ros::spinOnce();
		
		//want heding to update with last so were going to greenwich
		if(gps_fix || true)
		{
			//send the target to navigation
			if (inital_gps)
			{
				target_waypoint = find_target();
                odom_set_origin();
				inital_gps = false;
			}
						
			if(target_waypoint != -1)
			{
				target_heading = compute_msg(target_waypoint);
			}

			if(target_heading.distance <= params.waypoint_radius || skip_waypoint || (way_limit[target_waypoint] == 0 && target_heading.distance <= params.dummy_point_radius))
			{
				waypoint_complete[target_waypoint] = true;	
				target_waypoint = find_target();
				if(target_waypoint == -1)
				{
					//robot is done
					target_heading.done = true;
					if(params.continue_when_done)
					{
						target_heading.target_heading = pi/2;
						target_heading.distance = 2200000;
						
					}
					target_heading.stop_robot = false;
				}
				else
				{
					//go to next
					
					target_heading = compute_msg(target_waypoint);
					

				}
				skip_waypoint = false;
			}
			
			
			//just go straight
			if(!params.go_to_waypoints)
			{
	    		target_heading.target_heading = pi/2;
				target_heading.distance = 2200000;
				
			}
			
			if(way_limit[target_waypoint] == 0)
			{
			    target_heading.target_heading = pi/2;
			}

			target_pub.publish(target_heading);
            
            odom_pub.publish(odom_msg());
            
            goal_pub.publish(relative_waypoint(target_waypoint));
			
		}
		
		loop_rate.sleep();
    }
    
    return 0;
}

