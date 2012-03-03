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


//Subscriptions
image_transport::CameraSubscriber     image_sub_cam;

//Publications
image_transport::Publisher      image_pub_homography;

ros::Publisher                  laser_pub;

//camera model
image_geometry::PinholeCameraModel cam_model_;

//tranform listener
tf::TransformListener tf_listener_;



mst_homography::mst_homography_ParamsConfig params;

/***********************************************************
* Function prototypes
***********************************************************/

Mat find_perspective(float theta_x, float theta_y, float, theta_z));


/***********************************************************
* Defines
***********************************************************/

#define pi 3.14159265
