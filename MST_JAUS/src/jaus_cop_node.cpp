#include "ros/ros.h"
#include "JAUS_COP.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jaus_cop_node");
    ros::NodeHandle n;
    JAUS_COP jaus( n );
   
    ros::Rate loop_rate(2);    
 
    while( ros::ok() && jaus.run() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
