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
#include <geometry_msgs/Twist.h>
#include <MST_Position/Target_Heading.h>



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
#include <dynamic_reconfigure/server.h>
#include <mst_homography/mst_homography_ParamsConfig.h>
#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>

/***********************************************************
* Namespace Changes
***********************************************************/
namespace enc = sensor_msgs::image_encodings;


/***********************************************************
* Global variables
***********************************************************/

cv_bridge::CvImage              map;
cv_bridge::CvImage              map_dis;


cv::Mat                         stat;
cv::Mat                         edges;

/*
std::queue<cv::Mat>             stat_q;
std::queue<cv::Mat>             edges_q;
*/

image_transport::Subscriber     image_sub_edges;
image_transport::Subscriber     image_sub_line;
image_transport::Subscriber     image_sub_flags;
image_transport::Subscriber     image_sub_obst;
image_transport::Subscriber     image_sub_grass;
image_transport::Subscriber     image_sub_stat;


image_geometry::PinholeCameraModel cam_model_;



ros::Subscriber                 target_sub;

ros::Publisher                  twist_pub;
image_transport::Publisher      map_pub;

MST_Position::Target_Heading    target;

bool                            map_changed = 0;
std::queue<ros::Time>                       edges_time_q ;
std::queue<ros::Time>                       stat_time_q ;

bool                            first_callback = 1;

mst_homography::mst_homography_ParamsConfig params;

/***********************************************************
* Function prototypes
***********************************************************/
geometry_msgs::Twist find_twist();




/***********************************************************
* Defines
***********************************************************/

#define pi 3.14159265
