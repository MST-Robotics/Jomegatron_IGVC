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

sensor_msgs::LaserScan laserScan;

struct Ray
{
  int health;
  double angle;
  double x;
  double y;
};
Ray rays[LASER_SCAN_NUM_RAYS];

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_to_laser_scan");
    ros::NodeHandle n;
    
    ros::Rate loop_rate(2);   
    
    //Subscribe to the message we need
    ros::Subscriber s_image = n.subscribe("???", 1, &imageCallback);

    //Advertise our message
    p_laserScan = n.advertise<sensor_msgs::LaserScan>("/laser_scan", 5);
    
    ros::spin();
    
    s_image.shutdown();
    p_laserScan.shutdown();
    
    return 0;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    //TODO transform image
    
    //Generate laser scan
    double angleIncrement = LASER_SCAN_FOV / LASER_SCAN_NUM_RAYS;
    double startAngle = 0 - LASER_SCAN_FOV / 2.0;
    for(int i=0; i<LASER_SCAN_NUM_RAYS; i++)
    {
      rays[i].health = LASER_SCAN_HEALTH;
      rays[i].angle = startAngle + i * angleIncrement;
      rays[i].x = msg->width / 2; //image center
      rays[i].y = msg->height; //image bottom
    }
    //TODO while(rays not dead) move rays if alive
    
    //Publish laser scan
    p_laserScan.publish(laserScan);
}

