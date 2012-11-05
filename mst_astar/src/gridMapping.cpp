/*******************************************************************************
* @file gridMapping.cpp
* @author Jason Gassel <jmgkn6@mst.edu>
* @version 1.0
* @date 10/25/12
* @brief Generates a 2D map from laser scan and odom messages.
******************************************************************************/
#include "ros/ros.h"

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void positionCallback(const nav_msgs::Odometry::ConstPtr& msg);
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grid_mapping");
    ros::NodeHandle n;
   
    ros::Rate loop_rate(2);   
    
    //Subscribe to the messages we need
    ROS::Subscriber s_odom = n.subscribe("/odom", 1, &odomCallback, this);
    ROS::Subscriber s_position = n.subscribe("/position", 1, &positionCallback, this);
    ROS::Subscriber s_laserScan = n.subscribe("/laser_scan", 1, &laserScanCallback, this);

    //Advertise our message
    ROS::Publisher p_map = n.advertise<nav_msgs::OccupancyGrid>("/map", 5) 
 
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}


