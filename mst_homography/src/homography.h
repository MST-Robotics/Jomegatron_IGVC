/*******************************************************************************
 * @file homography.cpp
 * @author James Anderson <jra798>
 * @version 1.1
 * @date 2/21/12
 * @brief prforms homography transform to footprint and creates laser scan 
 ******************************************************************************/

/***********************************************************
* ROS specific includes
***********************************************************/
#include "ros/ros.h"

/***********************************************************
* Message includes
***********************************************************/
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>



/***********************************************************
* Other includes
***********************************************************/
#include <iostream>
#include <fstream>
#include <queue>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cv.h>
#include <highgui.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <mst_homography/homography_ParamsConfig.h>

/***********************************************************
* Namespace Changes
***********************************************************/
namespace enc = sensor_msgs::image_encodings;


/***********************************************************
* Global variables
***********************************************************/


//Subscriptions
image_transport::Subscriber     image_sub_color;
image_transport::Subscriber     image_sub_masked;

//Publications
image_transport::Publisher      image_pub_color;
image_transport::Publisher      image_pub_masked;

ros::Publisher                  laser_pub;

//transformation matrix
cv::Mat                         _transform; 


mst_homography::homography_ParamsConfig params;

/***********************************************************
* Function prototypes
***********************************************************/

cv::Mat find_perspective(float theta_x, float theta_y, float theta_z, float center_x,float center_y);


/***********************************************************
* Defines
***********************************************************/

#define PI 3.14159265
