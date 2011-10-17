#include "ros/ros.h"

#include "mst_midg/IMU.h"
#include "geometry_msgs/Twist.h"

struct waypoint_type
{
waypoint_type(double a, double b)
    {
    lat = a * M_PI/180.;
    lon = b * M_PI/180.;
    found = false;
    }
double lat;
double lon;
bool   found;
};

static double f_v, t_v, my_lat, my_lon, my_hdg;
std::vector<waypoint_type> waypoints;
ros::Publisher motor_pub;

bool active_waypoints()
{
    bool done = true;
    for( unsigned int i = 0; i < waypoints.size(); i++ )
        if( waypoints[i].found == false )
            done = false;
    return done;
}

//~ void gps_to_local( double& lat, double& lon )
//~ {
    //~ ROS_ERROR("GPS_TO_LOCAL IS NOT CORRECT! NEED TO FIX SCALES!");
    //~ ROS_INFO("Before: lat %f lon %f", lat, lon);
    //~ 
    //~ // Find a website / formula to convert lat/lon to meters
    //~ // lat * meters per degree + curent_pos_in_m
    //~ lat = lat*87890.32707564854+7900000;
    //~ lon = lon*110995.532350726862-3700000;
    //~ 
    //~ ROS_INFO("After:  lat %f lon %f", lat, lon);
//~ }
//~ 
//~ double compute_dist( double lat1, double lon1, double lat2, double lon2 )
//~ {
    //~ gps_to_local(lat1,lon1);
    //~ gps_to_local(lat2,lon2);
    //~ 
    //~ double delta_lat = lat2 - lat1;
    //~ double delta_lon = lon2 - lon1;
    //~ 
    //~ return sqrt( pow(delta_lat,2) + pow(delta_lon,2) );
//~ }
//~ 
//~ double compute_hdg( double lat1, double lon1, double lat2, double lon2 )
//~ {
    //~ gps_to_local(lat1,lon1);
    //~ gps_to_local(lat2,lon2);
    //~ 
    //~ double delta_lat = lat2 - lat1;
    //~ double delta_lon = lon2 - lon1;
    //~ 
    //~ double heading = atan2(delta_lon,delta_lat);
    //~ 
    //~ while( heading > M_2_PI ) heading -= M_2_PI;
    //~ while( heading < -M_2_PI) heading += M_2_PI;
    //~ 
    //~ return atan2(delta_lon,delta_lat);
//~ }

/***********************************************************
* @fn double find_dist(double lat1, double lon1 , double lat2 ,double lon2)
* @brief computes distace between two gps points
* @pre takes in lat and lon for two points
* @post returns a double with the distace in meeters between points
***********************************************************/
double compute_dist(double lat1, double lon1 , double lat2 ,double lon2)
{
    ROS_INFO("%f %f %f %f", lat1, lon1, lat2, lon2);
    double R = 6371000;
    double delta_lat = lat2 - lat1;
    double delta_lon = lon2 - lon1;
    double a = pow(sin(delta_lat/2.),2) + cos(lat1) * cos(lat2) * pow(sin(delta_lon/2.),2);
    double c =  2. * atan2(sqrt(a), sqrt(1 - a));

    return  R * c ;
}

/***********************************************************
* @fn double find_heading(double lat1, double lon1 , double lat2 ,double lon2)
* @brief computes heading between two gps points
* @pre takes in lat and lon for two points
* @post returns a double with the heading in radians
***********************************************************/
double compute_hdg(double lat1, double lon1 , double lat2 ,double lon2)
{
    double delta_lon = lon2 - lon1;
    double heading;

    double x = sin(delta_lon) * cos(lat2);

    double y = cos(lat1) * sin(lat2) -
        sin(lat1) * cos(lat1) * cos(delta_lon);
    double bearing = atan2(x,y);

    heading = -bearing + M_PI_2; 
    while( heading > M_2_PI ) heading -= M_2_PI;
    while( heading < M_2_PI ) heading += M_2_PI;

    return heading;
}

void MidgCallback(const mst_midg::IMU::ConstPtr& msg)
{
    static int active_waypoint = 0;
    
    if(active_waypoint >= 2)
        return;
        
    geometry_msgs::Twist out_twist;
    
    if( msg->position_valid )
    {
        my_lat = msg->latitude * M_PI/180.;
        my_lon = msg->longitude * M_PI/180.;
    }
    else
    {
        return;
    }
    my_hdg = msg->heading;
    
    if( waypoints[active_waypoint].found )
        if(active_waypoint++ >= 2)
        {
        out_twist.linear.y = f_v;
        out_twist.angular.z = t_v;
        motor_pub.publish(out_twist);
        return;
        }
        
    double target_hdg = compute_hdg(my_lat,my_lon,
        waypoints[active_waypoint].lat,
        waypoints[active_waypoint].lon);
    double target_dist = compute_dist(my_lat,my_lon,
        waypoints[active_waypoint].lat,
        waypoints[active_waypoint].lon);
        
    ROS_INFO("Approaching %d at brg %.9f, hdg %.9f, range %.9f", active_waypoint, target_hdg, (my_hdg-target_hdg), target_dist);
        
    if( target_dist < 2 )
    {
        ROS_INFO("Found %d at %.9f, %.9f", active_waypoint, waypoints[active_waypoint].lat, waypoints[active_waypoint].lon);
        waypoints[active_waypoint].found = true;
        f_v = 0;
        t_v = 0;
    }
    else
    {
        ROS_INFO("my_hdg %f target_hdg %f cos %f", my_hdg, target_hdg, cos(-my_hdg+target_hdg));
        t_v = 0.50 * ( -my_hdg + target_hdg );
        f_v = 1.0 * cos( -my_hdg + target_hdg );
        //~ f_v = 1.0;
    }
    
    ROS_INFO("twist y: %f z: %f", f_v, t_v);
    out_twist.linear.y = f_v;
    out_twist.angular.z = t_v;
    motor_pub.publish(out_twist);
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"simple_waypoints");
    ros::NodeHandle n;
    
    ros::Subscriber imu_sub = n.subscribe("/midg", 10, MidgCallback);
    motor_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    
    waypoints.push_back( waypoint_type(42.677930042,-83.195488794) ); // P14
    waypoints.push_back( waypoint_type(42.678193208,-83.195512597) ); // P13
    
    ROS_INFO("Waypoint list:");
    for( unsigned int i = 0; i < waypoints.size(); i++ )
    {
        ROS_INFO("\t%d: %.9f, %.9f", i, waypoints[i].lat, waypoints[i].lon);
    }
    
    ros::spin();
}
