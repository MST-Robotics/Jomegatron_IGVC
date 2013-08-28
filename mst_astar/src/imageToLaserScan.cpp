/*******************************************************************************
* @file imageToLaserScan.cpp
* @author Jason Gassel <jmgkn6@mst.edu>
* @version 1.0
* @date 1/14/12
* @brief Generates a laser scan message from a processed image.
******************************************************************************/

#include "ros/ros.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"

#include "constants.h"

void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

ros::Publisher p_laserScan;
ros::Publisher p_laserScanImage;

sensor_msgs::LaserScan laserScan;

struct Ray
{
  int health;
  double angle;
  double distanceTravelled;
  double x;
  double y;
  double diffX;
  double diffY;
};
Ray rays[LASER_SCAN_NUM_RAYS];

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_to_laser_scan");
    ros::NodeHandle n;
    
    ros::Rate loop_rate(2);   
    
    //Subscribe to the message we need
    ros::Subscriber s_image = n.subscribe("image_stat", 1, &imageCallback);

    //Advertise our message
    p_laserScan = n.advertise<sensor_msgs::LaserScan>("/laser_scan", 5);
    p_laserScanImage = n.advertise<sensor_msgs::Image>("/laser_scan_image", 5);
    
    ros::spin();
    
    s_image.shutdown();
    p_laserScan.shutdown();
    
    return 0;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    if(msg->encoding != "rbg8")
    {
        ROS_ERROR("Need rbg8 image encoding, not publishing laser scan.");
        return;
    }
    
    //TODO transform image
    
    //Init rays
    double angleIncrement = (LASER_SCAN_FOV*PI/180) / LASER_SCAN_NUM_RAYS;
    double startAngle = 0 - (LASER_SCAN_FOV*PI/180) / 2.0;
    for(int i=0; i<LASER_SCAN_NUM_RAYS; i++)
    {
      rays[i].health = LASER_SCAN_HEALTH;
      rays[i].angle = startAngle + i * angleIncrement;
      rays[i].x = msg->width / 2; //image center
      rays[i].y = msg->height; //image bottom
      rays[i].diffX = LASER_SCAN_RAY_STEP * sin(startAngle + angleIncrement*i);
      rays[i].diffY = LASER_SCAN_RAY_STEP * cos(startAngle + angleIncrement*i);
    }

    //Init debug image
    sensor_msgs::Image raycastImage;
    for(int i=0; i<msg->data.size(); i++)
        raycastImage.data.push_back(msg->data[i]);
    
    //Raycast
    int raysAlive = LASER_SCAN_NUM_RAYS;
    while(raysAlive > 0)
    {
        for(int i=0; i<LASER_SCAN_NUM_RAYS; i++)
        {
            //Move rays if still alive
            if(rays[i].health > 0)
            {
                rays[i].distanceTravelled += LASER_SCAN_RAY_STEP;
                rays[i].x += rays[i].diffX;
                rays[i].y += rays[i].diffY;
                
                int currentPixelIndex = (int)rays[i].y * msg->width * 3 + (int)rays[i].x * 3;
                unsigned char pixelData = msg->data[currentPixelIndex + 1]; //green channel
                if(pixelData > LASER_SCAN_PIXEL_THRESHOLD)
                {
                    rays[i].health -= 1;
                    if(rays[i].health <= 0)
                        raysAlive--;
                }
                
                //Debug image
                raycastImage.data[currentPixelIndex] = 255;
                raycastImage.data[currentPixelIndex + 2] = 0;
            }
        }
    }
    
    //Publish laser scan
    laserScan.header = msg->header;
    laserScan.angle_min = startAngle;
    laserScan.angle_max = startAngle + angleIncrement*(LASER_SCAN_NUM_RAYS-1);
    laserScan.angle_increment = angleIncrement;
    //laserScan.time_increment
    //laserScan.scan_time
    //laserScan.range_min
    //laserScan.range_max
    for(int i=0; i<LASER_SCAN_NUM_RAYS; i++)
    {
        laserScan.ranges.push_back(rays[i].distanceTravelled);
        laserScan.intensities.push_back(rays[i].health > 0 ? 1.0 : 0.0);//TODO function for intesities
    }
    p_laserScan.publish(laserScan);
    
    //Publish debug image
    raycastImage.height = msg->height;
    raycastImage.width = msg->width;
    raycastImage.encoding = msg->encoding;
    raycastImage.is_bigendian = msg->is_bigendian;
    raycastImage.step = msg->step;
    p_laserScanImage.publish(raycastImage);
}

