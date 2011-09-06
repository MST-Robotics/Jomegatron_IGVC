/*******************************************************************************
 * @file Vision.cpp
 * @author Chris Bessent <cmbq76>
 *
 * @brief Filter algorithm based on Hue Chroma and White derived
 *       from RGB values.
 ******************************************************************************/


/***********************************************************
* ROS specific includes
***********************************************************/
#include "ros/ros.h"

/***********************************************************
* Message includes
***********************************************************/
#include "sensor_msgs/Image.h"
#include "mst_common/VisionFilter.h"

/***********************************************************
* Other includes
***********************************************************/
#include "Configure.h"
#include <iostream>
#include <fstream>

/***********************************************************
* Global variables
***********************************************************/
mst_common::VisionFilter    g_current_filter;

unsigned char*              g_working_image;
unsigned int                g_working_image_height;
unsigned int                g_working_image_width;
unsigned int                g_working_image_step;

unsigned char*              g_output_image;

ros::Publisher              g_output_pub;

/***********************************************************
* Function prototypes
***********************************************************/
void initImage(const sensor_msgs::Image::ConstPtr&);
void applyFilters(void);
void readFiltersFromFile(void);
void saveFiltersToFile(void);

/***********************************************************
* Message Callbacks
***********************************************************/
void filterCallback(const mst_common::VisionFilter::ConstPtr& msg)
{
	ROS_INFO("filterCallback: New filters receieved");
    g_current_filter = *msg;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	ROS_INFO("imageCallback: Image receieved");
    initImage( msg );
    applyFilters();

    sensor_msgs::Image      output_msg;
    output_msg.height = g_working_image_height;
    output_msg.width  = g_working_image_width;
    output_msg.step   = g_working_image_width;
    output_msg.encoding = "mono8";
    for( unsigned int i = 0; i < g_working_image_height * g_working_image_width; i++ )
    {
        output_msg.data.push_back( g_output_image[i] );
    }

	ROS_INFO("imageCallback: Sending %dx%d / %d image",
		output_msg.width, output_msg.height, output_msg.step );
    g_output_pub.publish( output_msg );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision");
    ros::NodeHandle n;

    ros::Subscriber filter_sub = n.subscribe( FILTER_TOPIC, 1, filterCallback );
    ros::Subscriber camera_sub = n.subscribe( CAMERA_TOPIC, 1, imageCallback  );

    g_output_pub = (ros::Publisher)(n.advertise<sensor_msgs::Image>( FILT_IMAGE_TOPIC, 5 ));

    g_working_image           = NULL;
    g_working_image_height    = 0;
    g_working_image_width     = 0;
    
    readFiltersFromFile();

    ros::spin();
    
    saveFiltersToFile();

    return 0;
}

// Reallocates memory if image size has changed. Converts
// image to color system used to filter
void initImage(const sensor_msgs::Image::ConstPtr& msg)
{
	ROS_INFO("initImage:\t%dx%d", msg->width, msg->height);
    if( (g_working_image_height != msg->height) ||
        (g_working_image_width  != msg->width ) ||
        (g_working_image_step != msg->step  ) )
    {
		ROS_INFO("Allocating new image memory");
        if( g_working_image != NULL )
        {
            delete[] g_working_image;
        }
        if( g_output_image != NULL )
        {
            delete[] g_output_image;
        }

        g_working_image_height = msg->height;
        g_working_image_width  = msg->width;
        g_working_image_step   = msg->step;

        g_working_image   = new unsigned char[msg->height*msg->width*3];
        g_output_image    = new unsigned char[msg->height*msg->width*1];
    }

    unsigned int index = 0;
    unsigned char red, blue, green;
    double alpha, beta, result;
    const double angle = 255.0 / asin(0.5) / 12.0;

	ROS_INFO("initImage: Converting colorspace");
    for( unsigned int i = 0; i < msg->height; i++ )
    {
        for( unsigned int j = 0; j < msg->width; j++ )
        {
			if( msg->encoding == "bgr8" )
			{
				blue    = msg->data[index + 0];
				green   = msg->data[index + 1];
				red     = msg->data[index + 2];
			}
			else
			if( msg->encoding == "rgb8" )
			{
				blue    = msg->data[index + 2];
				green   = msg->data[index + 1];
				red     = msg->data[index + 0];
			}
			else
			if( msg->encoding == "mono8" )
			{
				blue    = msg->data[index + 0];
				green   = msg->data[index + 0];
				red     = msg->data[index + 0];
			}

            alpha   = red - ( blue + green )/2.0;
            beta    = sqrt(3)/2.0 * ( green - blue );

            result = atan2(beta, alpha) * angle;
            g_working_image[index + 0] = (unsigned char)( result );

            result = sqrt( pow(alpha,2) + pow(beta,2) );
            g_working_image[index + 1] = (unsigned char)( result );

            result = std::min( blue, std::min( green, red ) );
            g_working_image[index + 2] = (unsigned char)( result );
            
            index += 3;
        }
    }
}

void applyFilters( void )
{
	ROS_INFO("applyFilters: Applying filters");
    unsigned int index = 0;
    unsigned char hue, chroma, white;
    double hue_gain, chroma_gain, white_gain;

    for( unsigned int i = 0; i < g_working_image_height; i++ )
    {
        for( unsigned int j = 0; j < g_working_image_width; j++ )
        {
            hue     = g_working_image[ index++ ];
            chroma  = g_working_image[ index++ ];
            white   = g_working_image[ index++ ];

            hue_gain    = 1.0;
            chroma_gain = 1.0;
            white_gain  = 1.0;

            for( unsigned int k = 0; k < g_current_filter.color.size(); k++ )
            {
                hue_gain    = hue_gain * (g_current_filter.color[k].filter[0].gain[hue]/255.0);
                chroma_gain = chroma_gain * (g_current_filter.color[k].filter[1].gain[chroma]/255.0);
                white_gain  = white_gain * (g_current_filter.color[k].filter[2].gain[white]/255.0);
            }

            g_output_image[ i*g_working_image_width + j ] =
                (unsigned char)( 255 * hue_gain * chroma_gain * white_gain );
        }
    }
}

void readFiltersFromFile()
{
    std::ifstream filter_file;
    filter_file.open( "filter.bin", std::ios::binary );
    if( filter_file.is_open() )
    {
        g_current_filter.color.clear();
        
        int num_filters;
        mst_common::ImageFilter image_filter;
        
        filter_file >> num_filters;
        if( filter_file.good() )
        {
            for( int i = 0; i < num_filters; i++ )
            {
                for( int j = 0; j < 3; j++ )
                {
                    for( int k = 0; k < 256; k++ )
                    {
                        filter_file >> image_filter.filter[j].gain[k];
                    }
                }
                g_current_filter.color.push_back(image_filter);
            }
        }
        filter_file.close();
    }
}

void saveFiltersToFile()
{
    std::ofstream filter_file;
    filter_file.open( "filter.bin", std::ios::binary );
    if( filter_file.is_open() )
    {
        int num_filters = g_current_filter.color.size();
        filter_file << num_filters << " ";
        
        if( filter_file.good() )
        {
            for( int i = 0; i < num_filters; i++ )
            {
                for( int j = 0; j < 3; j++ )
                {
                    for( int k = 0; k < 256; k++ )
                    {
                        filter_file << g_current_filter.color[i].filter[j].gain[k];
                    }
                }
            }
        }
        filter_file.close();
    }
}
