#include "ros/ros.h"
#include "JAUS_Controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jaus_node");
    ros::NodeHandle n;
    JAUS_Controller jaus( n );
   
    ros::Rate loop_rate(2);    
 
    while( ros::ok() && jaus.run() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
