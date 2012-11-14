/*******************************************************************************
* @file gridMapping.cpp
* @author Jason Gassel <jmgkn6@mst.edu>
* @version 1.0
* @date 10/25/12
* @brief Generates a 2D grid map from laser scan and odom messages.
******************************************************************************/
#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

#include "constants.h"

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

nav_msgs::Odometry odom;
sensor_msgs::LaserScan laserScan;
nav_msgs::OccupancyGrid grid;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grid_mapping");
    ros::NodeHandle n;
   
    ros::Rate loop_rate(2);   
    
    //Subscribe to the messages we need
    ros::Subscriber s_odom = n.subscribe("robot_pose_ekf/odom_combined", 1, &odomCallback);
    ros::Subscriber s_laserScan = n.subscribe("/laser_scan", 1, &laserScanCallback);

    //Advertise our message
    ros::Publisher p_map = n.advertise<nav_msgs::OccupancyGrid>("/map", 5); 
 
    //Initialize our occupancy grid message
    grid.info.resolution = MAP_RESOLUTION;
    grid.info.width = MAP_WIDTH;
    grid.info.height = MAP_HEIGHT;
    grid.info.origin.position.x = MAP_ORIGIN_POSITION_X;
    grid.info.origin.position.y = MAP_ORIGIN_POSITION_Y;
    grid.info.origin.position.z = MAP_ORIGIN_POSITION_Z;
    grid.info.origin.orientation.x = MAP_ORIGIN_ORIENTATION_X;
    grid.info.origin.orientation.y = MAP_ORIGIN_ORIENTATION_Y;
    grid.info.origin.orientation.z = MAP_ORIGIN_ORIENTATION_Z;
    grid.info.origin.orientation.w = MAP_ORIGIN_ORIENTATION_W;
    for(unsigned int i=0; i<MAP_WIDTH*MAP_HEIGHT; i++)
        grid.data.push_back(1);

    while(ros::ok())
    {
        //Update map
        double robotAngle = odom.pose.pose.orientation.z * PI; //convert unit vector to radians
        for(unsigned int i=0; i<laserScan.ranges.size(); i++)
        {
            if(laserScan.intensities[i] > 0)
            {
                //Potential obstacle detected
                double angle = laserScan.angle_min + i * laserScan.angle_increment;
                double x = odom.pose.pose.position.x + laserScan.ranges[i] * cos(angle + robotAngle);
                double y = odom.pose.pose.position.x + laserScan.ranges[i] * cos(angle + robotAngle);
                int gridX = x / MAP_RESOLUTION; //TODO should be * resolution instead? idk
                int gridY = y / MAP_RESOLUTION;
                grid.data[gridX+gridY*MAP_WIDTH] += 1; //TODO actually figure out good way to set
            }
            //TODO may want to update map with nothing there to reduce error?
        }
        
        //Publish map
        p_map.publish(grid);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    s_odom.shutdown();
    s_laserScan.shutdown();
    p_map.shutdown();
    
    return 0;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom = *msg;
}

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laserScan = *msg;
}

