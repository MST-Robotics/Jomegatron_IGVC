/*******************************************************************************
* @file Pot_Nav.cpp
* @author James Anderson <jra798>
* @version 1.0
* @brief findes the forward and angular velocity to move the robot at 
* given a fuzzy model
******************************************************************************/


/***********************************************************
* ROS specific includes
***********************************************************/
#include "ros/ros.h"

/***********************************************************
* Message includes
***********************************************************/
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>

/***********************************************************
* Other includes
***********************************************************/
#include <iostream>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dynamic_reconfigure/server.h>
#include <MST_Potential_Navigation/Pot_Nav_ParamsConfig.h>

/***********************************************************
* Global variables
***********************************************************/

cv_bridge::CvImage              map;
cv_bridge::CvImage              map_dis;

image_transport::Subscriber     image_sub_edges;
image_transport::Subscriber     image_sub_line;
image_transport::Subscriber     image_sub_flags;
image_transport::Subscriber     image_sub_obst;
image_transport::Subscriber     image_sub_grass;

ros::Publisher                  twist_pub;
image_transport::Publisher      map_pub;

bool                            map_changed = 0;
bool                            first_callback = 1;

MST_Potential_Navigation::Pot_Nav_ParamsConfig params;

/***********************************************************
* Function prototypes
***********************************************************/
geometry_msgs::Twist find_twist();


/***********************************************************
* Namespace Changes
***********************************************************/
namespace enc = sensor_msgs::image_encodings;

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
void edgesCallback( const sensor_msgs::ImageConstPtr& msg)
{
	ROS_INFO("Pot_Nav: Edges image receieved");
	
	cv_bridge::CvImagePtr cv_ptr_src;
	
	
	//takes in the image
    try
    {
      cv_ptr_src = cv_bridge::toCvCopy(msg, "32FC1");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    if(first_callback)
    {
		//initalize map
		map.image = cv::Mat::zeros(cv_ptr_src->image.size(),CV_32FC1);
		map.encoding = "32FC1";
		
		first_callback = 0;
	}

    
    //adds them together using weghted persentage
    cv::addWeighted(cv_ptr_src->image, params.edges_per/100 , map.image, (100-params.edges_per)/100,1, map.image);
    
    map.header = cv_ptr_src->header;
    
    map_changed = 1;

}

/***********************************************************
* @fn setparamsCallback(const sensor_msgs::ImageConstPtr& msg)
* @brief callback for the reconfigure gui
* @pre has to have the setup for the reconfigure gui
* @post changes the parameters
***********************************************************/
void setparamsCallback(MST_Potential_Navigation::Pot_Nav_ParamsConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request : %i %i %f %f %f %f %f ",
           config.robot_x, config.robot_y, config.edges_per, config.lines_per, config.flags_per, 
           config.obst_per, config.grass_per);
  
  // set params
  params = config;
  
}

/***********************************************************
* Private Functions
***********************************************************/

/***********************************************************
* @fn find_twist()
* @brief computes the twists using the map
* @pre needs a globaly defined map it uses to compute twist
* @post computes the twist
* @return reterns the gemoetry messages standard twist and 
* draws debug info on map_dis
***********************************************************/
geometry_msgs::Twist find_twist()
{
	cv::Point2i robot_center;
    geometry_msgs::Twist twist;
    cv::Point2i box_1;
    cv::Point2i box_2;
    
    
    //find botom center of the image
    robot_center.x = map.image.cols/2 ;
    robot_center.y = map.image.rows - 1 ;
    
    box_1.x = robot_center.x - params.robot_x;
    box_1.y = robot_center.y - params.robot_y;
    
    box_2.x = robot_center.x + params.robot_x;
    box_2.y = robot_center.y + params.robot_y;
    
    map.image.copyTo(map_dis.image);
    map_dis.header = map.header;
    map_dis.encoding = map.encoding;
    
    //draw box in place of robot
	cv::rectangle(map_dis.image, box_1, box_2 , 255 ,-2*params.robot_filled+1);
	
	//remove robot from the map
	cv::rectangle(map.image, box_1, box_2 , 0,-1);
	
	//draw the search radius
	cv::circle(map_dis.image, robot_center, params.search_radius, 255);
	
    //initalize twist and apply forward foring funciton
	twist.linear.x = 0;
	twist.linear.y = params.carrot_on_a_stick;
    twist.linear.z = 0;
    
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    
    
    //compute twist
    for(int deg = 0 ;  deg <= params.search_res ; deg++ )
    {
		
		float degree = deg;
		float res = params.search_res;
		float theta = pi*(degree/res);
		cv::Point2f search_edge;
		
		
		//compute the edge point
		search_edge.x = (robot_center.x + ((cos(theta) * params.search_radius)));
		search_edge.y = (robot_center.y - ((sin(theta) * params.search_radius)));
		
		//draw rays
		/*
		if(params.display_rays)
		{
			line(map_dis.image, robot_center, search_edge , 255);
		}
		
		*/
		
		cv::LineIterator ray(map.image, robot_center, search_edge , 8);
		
		for(int pos = 0 ; pos < ray.count ; pos++ , ++ray)
		{

			    
			if(ray.pos().x > robot_center.x + params.robot_x || 
			   ray.pos().x < robot_center.x - params.robot_x ||
			   ray.pos().y < robot_center.y - params.robot_y )
			{
			
			
				/*
				@todo This is most definatly the wrong way to access pixls on the iterator sice it should return a pointer
				but I give up for now
				*/
				if(params.display_rays)
				{
					map_dis.image.at<float>( ray.pos().y , ray.pos().x ) = 200; 
				}
				
				
			    //magnitued divided by the distance squared 
			    double dist = pow(abs(ray.pos().x - robot_center.x ),2) + pow(abs(ray.pos().y - robot_center.y ),2);
			    float mag = map.image.at<float>( ray.pos().y , ray.pos().x );
                
                
			    
				twist.linear.y -= (params.twist_scalar_y/100  * mag/dist * sin(theta));
				twist.angular.z += (params.twist_scalar_z/100  * mag/dist * cos(theta));	   

 			} 
 			//twist.linear.y = twist.linear.y*100/params.search_res ;
 			//twist.angular.z = twist.angular.z*100/params.search_res ;
 			
		}
		
		
	}
       
    
    return twist; 
}


/***********************************************************
* @fn main(int argc, char **argv)
* @brief starts the Pot_Nav node and publishises twist when 
* it gets a new image asuming 30 hz
***********************************************************/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Pot_Nav");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    
    
	dynamic_reconfigure::Server<MST_Potential_Navigation::Pot_Nav_ParamsConfig> srv;
    dynamic_reconfigure::Server<MST_Potential_Navigation::Pot_Nav_ParamsConfig>::CallbackType f;
    f = boost::bind(&setparamsCallback, _1, _2);
	srv.setCallback(f);

	//create subsctiptions
    image_sub_edges = it.subscribe( n.resolveName("image_edges") , 1, edgesCallback );

    //create publishers
    twist_pub = n.advertise<geometry_msgs::Twist>( "nav_twist" , 5 );
    map_pub = it.advertise( "map" , 5 );
    
    //set rate to 30 hz
    ros::Rate loop_rate(30);
    
    
    
    //run main loop
	while (ros::ok())
    {
		//check calbacks
		ros::spinOnce();
		
		//finds and publishes new twist
		if(map_changed)
		{
			geometry_msgs::Twist twist;
			
			twist = find_twist(); 
			
			//publish twist
			twist_pub.publish(twist);
			
			//publish map
			map_pub.publish(map_dis.toImageMsg());
			
			map_changed = 0;
		}
		
		loop_rate.sleep();
    }
    
    return 0;
}

