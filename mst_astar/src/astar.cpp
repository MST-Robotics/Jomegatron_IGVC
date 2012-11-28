/*******************************************************************************
* @file astar.cpp
* @author 
* @version 1.0
* @date 11/27/2012
* @brief Generates a twist message using the A* algorithm on a grid map.
******************************************************************************/
#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Twist.h"

#include "constants.h"

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

nav_msgs::Odometry odom;
nav_msgs::OccupancyGrid grid;
geometry_msgs::Twist twist;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "astar");
    ros::NodeHandle n;
   
    ros::Rate loop_rate(2);   
    
    //Subscribe to the messages we need
    ros::Subscriber s_odom = n.subscribe("robot_pose_ekf/odom_combined", 1, &odomCallback);
    ros::Subscriber s_map = n.subscribe("/map", 1, &mapCallback);

    //Advertise our message
    ros::Publisher p_twist = n.advertise<geometry_msgs::Twist>("/nav_twist", 5); 

    while(ros::ok())
    {
        //TODO
        
        //Publish twist
        p_twist.publish(twist);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    s_odom.shutdown();
    s_map.shutdown();
    p_twist.shutdown();
    
    return 0;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom = *msg;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    grid = *msg;
}

