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
#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dynamic_reconfigure/server.h>
#include <mst_color/color_paramsConfig.h>
#include <vector>
#include <stdlib.h>

/***********************************************************
* Structs
***********************************************************/

//structure to keep track of pixel data
struct pixel
{
    int x;
    int y;
    struct run_length *x_parent;
    struct run_length *y_parent;
};

//structure to store average pixel values
struct average
{
    double x;
    double y;
    double chanel[4];
};

//structure to store run length data
struct run_length
{
    pixel *start;
    pixel *end;
    std::vector<pixel*> members;
    average avg;
    bool linked;
};

//structuree to store data for a segment
struct segment
{
    std::vector<pixel*> peremeter;
    std::vector<pixel*> members;
    average avg;
};


/***********************************************************
* Global variables
***********************************************************/

sensor_msgs::Image              image;


image_transport::Subscriber     image_sub;
image_transport::Publisher      dbg_pub;



mst_color::color_paramsConfig params;

/***********************************************************
* Function prototypes
***********************************************************/
void link_runs(std::vector<run_length>* , std::vector<run_length>* ,int , int , pixel*, segment* ,run_length*);


/***********************************************************
* Namespace Changes
***********************************************************/
namespace enc = sensor_msgs::image_encodings;


/***********************************************************
* Message Callbacks
***********************************************************/
