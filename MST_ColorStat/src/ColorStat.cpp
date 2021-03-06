/*******************************************************************************
 * @file Edge_Detection.cpp
 * @author James Anderson <jra798>
 *
 * @brief finds edges in image using sobel and then outputs a greyscale image
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

/***********************************************************
* Other includes
***********************************************************/
#include <iostream>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

#include <dynamic_reconfigure/server.h>
#include <MST_ColorStat/ColorStat_ParamsConfig.h>

#include "stat.h"
#include "error.h"

/***********************************************************
* Global variables
***********************************************************/

Stat stat1;
error error1;

//~ sensor_msgs::Image              image;

unsigned char*                  g_output_image;

image_transport::Subscriber     image_sub;
image_transport::Publisher      image_pub;

std::string                     topic;

MST_ColorStat::ColorStat_ParamsConfig params;


/***********************************************************
* Function prototypes
***********************************************************/

/***********************************************************
* Namespace Changes
***********************************************************/
namespace enc = sensor_msgs::image_encodings;


/***********************************************************
* Message Callbacks
***********************************************************/

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//ROS_INFO("Edge_Detection: Image receieved");
	
	cv_bridge::CvImagePtr cv_ptr_src;
	
	
    try
    {
      cv_ptr_src = cv_bridge::toCvCopy(msg, "rgb8");
      
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    image input;
    unsigned char* ptr = cv_ptr_src->image.data;
    for(int i = 0; i < IMAGE_SIZE; i++)
    {
      *( (unsigned char*)input.rgb + i ) = *( ptr + i);
    }
    
    stat1.in = &input;
    stat1.update();
    
    error1.in = &input;
    error1.stats = &stat1;
    error1.update();
    
    cv::Mat out(cv_ptr_src->image.size(), CV_8UC3);
    cv_bridge::CvImage out_msg;
    
    out = cv::Mat::ones(cv_ptr_src->image.size(),CV_8UC3);
    
    unsigned char* fptr = out.data;
    for(int i = 0; i < IMAGE_SIZE; i+=3)
    {
      *( fptr + i + 1) = *( (unsigned char*) error1.out2.rgb + i + 1 );
    }
    
    //draw lines
    //~ for(int i = stat1.xMin; i < stat1.xMax; i++) 
    //~ {
        //~ error1.out2.rgb[stat1.yMin][i][2] = 255;
        //~ error1.out2.rgb[stat1.yMax][i][2] = 255;
    //~ }
    //~ for(int j = stat1.yMin; j < stat1.yMax; j++)
    //~ {
        //~ error1.out2.rgb[j][stat1.xMin][2] = 255;
        //~ error1.out2.rgb[j][stat1.xMax][2] = 255;
    //~ }
	cv::rectangle(out, cv::Point(stat1.xMin,480 - stat1.yMax), cv::Point(stat1.xMax,480 - stat1.yMin), cv::Scalar(0,0,255) ,3);    
	
    out_msg.header = cv_ptr_src->header;
    out_msg.encoding = "rgb8";
    out_msg.image = out ;
    
    image_pub.publish(out_msg.toImageMsg());

}

/***********************************************************
* @fn setparamsCallback(const sensor_msgs::ImageConstPtr& msg)
* @brief callback for the reconfigure gui
* @pre has to have the setup for the reconfigure gui
* @post changes the parameters
***********************************************************/
void setparamsCallback(MST_ColorStat::ColorStat_ParamsConfig &config, uint32_t level)
{
  
  
  stat1.setBounds(config.xMin,config.xMax,config.yMin,config.yMax);
  stat1.enabled = config.enabled;
  
  if (config.reset == true)
  {
    stat1.reset = true;
    config.reset = false;
  }
  
  if(config.save)
  {
     stat1.saveFilter(config.filename.c_str());
     config.save = false;
  }

  if(config.load)
  {
    stat1.loadFilter(config.filename.c_str());
    config.xMin = stat1.xMin;
    config.yMin = stat1.yMin;
    config.xMax = stat1.xMax;
    config.yMax = stat1.yMax;
    config.reset = stat1.reset;
    config.enabled = stat1.enabled;
    config.load = false;
  }
  
  // set params
  params = config;
  
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ColorStat");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    
    //setup dynamic reconfigure gui
    dynamic_reconfigure::Server<MST_ColorStat::ColorStat_ParamsConfig> srv;
    dynamic_reconfigure::Server<MST_ColorStat::ColorStat_ParamsConfig>::CallbackType f;
    f = boost::bind(&setparamsCallback, _1, _2);
	srv.setCallback(f);
    
    //get topic name
    topic = n.resolveName("image");

	//check to see if user has defined an image to subscribe to 
    if (topic == "/image") 
    {
		ROS_WARN("Edge_Detection: image has not been remapped! Typical command-line usage:\n"
				 "\t$ ./Edge_Detection image:=<image topic> [transport]");
    }
    

    image_sub = it.subscribe( topic , 1, imageCallback  );

    image_pub = it.advertise( "image_stat" , 5 );

    ros::spin();
    
    return 0;
}

