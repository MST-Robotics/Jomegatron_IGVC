/*******************************************************************************
* @file Pot_Nav.cpp
* @author James Anderson <jra798>
* @version 1.0
* @brief findes the forward and angular velocity to move the robot at 
* given a fuzzy model
******************************************************************************/
#include "homography.h"



/***********************************************************
* Message Callbacks
***********************************************************/


//calback for color image
void homographyColorCallback( const sensor_msgs::ImageConstPtr& image_sub_msg)
{
    //ROS_INFO("Pot_Nav: Statistics image receieved");

    cv_bridge::CvImagePtr cv_ptr_src; // source image pointer
    cv_bridge::CvImage           dst; // destination image
    cv::Point2i robot_center;         //center of the robot
    cv::Point2i box_1;
    cv::Point2i box_2;
    
    
    
    //crete pointer to source image
    try
    {
      cv_ptr_src = cv_bridge::toCvCopy(image_sub_msg, "rgb8");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    

    //pull in center
    robot_center.x = params.center_x;
    robot_center.y = params.center_y;

    //setup dst
    dst.encoding = "rgb8";
    dst.header = cv_ptr_src->header;
    
    
    //transform image
    warpPerspective(cv_ptr_src->image, dst.image, _transform, cv_ptr_src->image.size() * (params.scale));
    
    
    //draw all debug data
    
    //draw a box in place of the robot
    box_1.x = robot_center.x - params.robot_x;
    box_1.y = robot_center.y - params.robot_y;
    
    box_2.x = robot_center.x + params.robot_x;
    box_2.y = robot_center.y + params.robot_y;
    
    cv::rectangle(dst.image, box_1, box_2 , cv::Scalar(0,0,255) ,0);
    
    //draw the calibration box
    box_1.x = robot_center.x - (((params.calibration_width/2) - params.calibration_x) * params.pixels_per_meter);
    box_1.y = robot_center.y - (((params.calibration_height) + params.calibration_y) * params.pixels_per_meter);
    
    box_2.x = robot_center.x + (((params.calibration_width/2) + params.calibration_x) * params.pixels_per_meter);
    box_2.y = robot_center.y - (params.calibration_y * params.pixels_per_meter);
    
    cv::rectangle(dst.image, box_1, box_2 , cv::Scalar(255,0,0) ,0);
    
    //draw robot center
    cv::circle(dst.image, robot_center, 5, cv::Scalar(0,0,255));

    //draw the search radius
    cv::circle(dst.image, robot_center, (params.laser_range_max * params.pixels_per_meter), 255);
    
    //publish the image
    image_pub_color.publish(dst.toImageMsg());

}

//calback for color image
void homographyMaskedCallback( const sensor_msgs::ImageConstPtr& image_sub_msg)
{
    //ROS_INFO("Pot_Nav: Statistics image receieved");

    cv_bridge::CvImagePtr cv_ptr_src; // source image pointer
    cv_bridge::CvImage           dst; // destination image
    cv::Point2i         robot_center; // holds the center of the robot
    std::vector<cv::Mat> Chanels;
    cv::Point2i box_1;
    cv::Point2i box_2;

    //camera model
    //image_geometry::PinholeCameraModel cam_model;

    //tranform listener
    //tf::TransformListener tf_listener;

    //crete pointer to source image
    try
    {
      cv_ptr_src = cv_bridge::toCvCopy(image_sub_msg, "rgb8");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    /*
    if(params.use_trans)
    {
        //get camera model from message
        cam_model.fromCameraInfo(info_msg);
        
        //crate transform to ground frame
        tf::StampedTransform transform;
        
        //get the transform between the camera frame and target frame
        try {
        ros::Time acquisition_time = info_msg->header.stamp;
        ros::Duration timeout(1.0 );
        tf_listener.waitForTransform(cam_model.tfFrame(), params.frame_id,
                                      ros::Time(0), timeout);
        tf_listener.lookupTransform(cam_model.tfFrame(), params.frame_id,
                                     ros::Time(0), transform);
        }
        catch (tf::TransformException& ex) {
            ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
            return;
        }
        
        //find the roll pitch and yaw
        float alpha, theta_x, theta_y, theta_z;
        
        alpha = 2*acos(transform.getRotation().w());
        theta_x = acos(transform.getRotation().x()/sin(alpha/2));
        theta_y = acos(transform.getRotation().y()/sin(alpha/2));
        theta_z = acos(transform.getRotation().z()/sin(alpha/2));
        
        //find the transform matrix
        T = find_perspective(theta_x, theta_y, theta_z, 
                             params.center_x, params.center_y);
    }
    */
        /* This is the start of a hacky way where 4 imaginary points on the ground 
    plain are used to find the transform it is not recomended
    //arrays of perpective points for perspective and raw images
    cv::Point2d32f objPts[4], imgPts[4];
    
    
    for(int i; i<4, i++)
    {
        //point on the ground plane
        tf::Point pt();
        //point transformed to camera frame
        tf::Point pt_tf = transform();
        //changed to cv point
        cv::Point3d pt_cv(pt_tf.x(), pt_tf.y(), pt.z());
        //2D point
        cv::Point2d uv;
        //find the point in the image
        cam_model.project3dToPixel(pt_cv, uv);
    }
    cv::getPerspectiveTransform()
    */
    
    
    //pull in center
    robot_center.x = params.center_x;
    robot_center.y = params.center_y;
    
    //setup dst
    dst.encoding = "rgb8";
    dst.header = cv_ptr_src->header;
    
    //transform image
    warpPerspective(cv_ptr_src->image, dst.image, _transform, cv_ptr_src->image.size() * (params.scale));
    
    //compute the laser scan
    sensor_msgs::LaserScan laser_scan;
    
    laser_scan.header = dst.header;
    laser_scan.header.frame_id = params.laser_frame;
    
    laser_scan.angle_min = -(params.laser_angle/2);
    laser_scan.angle_max = (params.laser_angle/2);
    laser_scan.angle_increment = (params.laser_angle/params.laser_res);
    
    laser_scan.time_increment = params.laser_time_increment;
    laser_scan.scan_time = params.laser_scan_time;
    
    laser_scan.range_min = params.laser_range_min;
    laser_scan.range_max = params.laser_range_max;
    
    
    
    //split chanels
    cv::split(cv_ptr_src->image , Chanels);
    
    Chanels[1].convertTo(Chanels[1],CV_32FC1);
    
    double ranges[params.laser_res];
    
    //fill ranges
    for(int i = 0; i <= params.laser_res; i++)
    {
        
        float theta = (PI/2 - (params.laser_angle/2)) + (i * (params.laser_angle/params.laser_res));
        cv::Point2f search_edge;
        
        
        //clear range
        ranges[i] = 0;
        float win_avg = 0;
        
        //compute the edge point
        search_edge.x = (robot_center.x + ((cos(theta) * params.laser_range_max * params.pixels_per_meter)));
        search_edge.y = (robot_center.y - ((sin(theta) * params.laser_range_max * params.pixels_per_meter)));
        
        
        cv::LineIterator ray(dst.image, robot_center, search_edge , 8);
        
        cv::LineIterator window(dst.image, robot_center, search_edge , 8);
        
        //move window window size ahead
        for(int win=0; win < params.window_size; win++ , ++window)
        {   
            
            win_avg += dst.image.at<float>( window.pos().y , window.pos().x , 1);
            
        }
        //undo first operation
        win_avg -= dst.image.at<int>( window.pos().y , window.pos().x , 1);
        win_avg += dst.image.at<int>( ray.pos().y , ray.pos().x , 1);
        
        for(int pos=0; pos < ray.count; pos++ , ++ray, ++window)
        {
            
            win_avg += dst.image.at<int>( window.pos().y , window.pos().x , 1);
            win_avg -= dst.image.at<int>( ray.pos().y , ray.pos().x , 1);
            
            //outside of box
            if((ray.pos().x > robot_center.x + params.robot_x || 
               ray.pos().x < robot_center.x - params.robot_x ||
               ray.pos().y < robot_center.y - params.robot_y ) /*&& 
               (ray.pos().x < dst.image.cols &&
                ray.pos().x >= 0)  &&
                ray.pos().y < dst.image.rows &&
                ray.pos().y >= 0)*/)
            {
                

                //see if its an obsticale
                if ((win_avg/params.window_size) >= params.laser_threshold)
                {
                    ROS_INFO("threshold: %f ", win_avg/params.window_size);
                    
                    //mark the obsticle
                    dst.image.at<int>( ray.pos().y , ray.pos().x , 0) = 200;
                    
                    //give the distance in meters
                    ranges[i] = (sqrt(pow(ray.pos().x,2) + sqrt(pow(ray.pos().y,2)) ) / params.pixels_per_meter);
                    
                    //done with this ray
                    break;
                }
                else
                {
                    //display the ray
                    dst.image.at<int>( ray.pos().y , ray.pos().x , 2) = 200;
                }
            }
            
        }
    }
    
    
    for(int i = 0; i <= params.laser_res; i++)
    {
        laser_scan.ranges.push_back( ranges[i]);
        
    }
    
    //publish the laser scan
    laser_pub.publish(laser_scan);
    
    //draw the search radius
    cv::circle(dst.image, robot_center, (params.laser_range_max * params.pixels_per_meter), cv::Scalar(0,0,255));
    
    //draw a box in place of the robot
    box_1.x = robot_center.x - params.robot_x;
    box_1.y = robot_center.y - params.robot_y;
    
    box_2.x = robot_center.x + params.robot_x;
    box_2.y = robot_center.y + params.robot_y;
    
    cv::rectangle(dst.image, box_1, box_2 , cv::Scalar(0,0,255) ,0);
    
    image_pub_masked.publish(dst.toImageMsg());

}

//this function finds the perspective transform matrix given the angle between the two frames
cv::Mat find_perspective(float theta_x, float theta_y, float theta_z,float center_x,float center_y)
{
    //these are the matricies for the transform they could be combined but 
    //I dont have my calculator or matlab and dont feel linke doing it by hand
    
    //these are the transposes
     cv::Mat A = (cv::Mat_<float>(3, 3) <<
     1, 0, 0,
     0, cos(theta_x), sin(theta_x),
     0, -sin(theta_x),  cos(theta_x));
    
     cv::Mat B = (cv::Mat_<float>(3, 3) <<
     cos(theta_y), 0, -sin(theta_y),
     0, 1, 0, 
     sin(theta_y), 0, cos(theta_y));
    
     cv::Mat C = (cv::Mat_<float>(3, 3) <<
     cos(theta_z), sin(theta_z), 0,
     -sin(theta_z),  cos(theta_z), 0, 
     0, 0, 1);
     
     //shift to center
     cv::Mat D = (cv::Mat_<float>(3, 3) <<
     1, 0, -center_x,
     0, 1, -center_y, 
     0, 0, 1);
     
     cv::Mat T = A*B*C*D;
     
     //shift back by sclaed ammounts
     cv::Mat E = (cv::Mat_<float>(3, 3) <<
     (T.at<float>(2,2) / T.at<float>(0,0) ), 0, center_x  ,
     0, (T.at<float>(2,2) / T.at<float>(1,1) ), center_y  , 
     0, 0, 1);
     
     T = E*T;
     
     
     
     return T;
}



/***********************************************************
* @fn setparamsCallback(const sensor_msgs::ImageConstPtr& msg)
* @brief callback for the reconfigure gui
* @pre has to have the setup for the reconfigure gui
* @post changes the parameters
***********************************************************/
void setparamsCallback(mst_homography::homography_ParamsConfig &config, uint32_t level)
{

  // set params
  params = config;
  
    _transform = find_perspective(params.theta_x, params.theta_y, params.theta_z, params.center_x, params.center_y);
  
}

/***********************************************************
* Private Functions
***********************************************************/



/***********************************************************
* @fn main(int argc, char **argv)
* @brief starts the Pot_Nav node and publishises twist when 
* it gets a new image asuming 30 hz
***********************************************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "homography");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    
    //setup dynamic reconfigure
    dynamic_reconfigure::Server<mst_homography::homography_ParamsConfig> srv;
    dynamic_reconfigure::Server<mst_homography::homography_ParamsConfig>::CallbackType f;
    f = boost::bind(&setparamsCallback, _1, _2);
    srv.setCallback(f);
    
    
    std::string topic = n.resolveName("image_color");
    
    //check to see if user has defined an image to subscribe to 
    if (topic == "/image_color") 
    {
        ROS_WARN("homography: image has not been remapped! Typical command-line usage:\n"
                "\t$ ./Edge_Detection image_color:=<image topic> [transport]");
    }
    
    topic = n.resolveName("image_masked");
    
    //check to see if user has defined an image to subscribe to 
    if (topic == "/image_masked") 
    {
        ROS_WARN("homography: image has not been remapped! Typical command-line usage:\n"
                "\t$ ./Edge_Detection image_masked:=<image topic> [transport]");
    }
    

    //create subsctiptions
    image_sub_color = it.subscribe( n.resolveName("image_color"), 1, homographyColorCallback);
    image_sub_masked = it.subscribe( n.resolveName("image_masked"), 1, homographyMaskedCallback);
    

    //create publishers
    laser_pub = n.advertise<sensor_msgs::LaserScan>( "homography_scan" , 5 );
    
    image_pub_color = it.advertise( "homography_color" , 1 );
    image_pub_masked = it.advertise( "homography_masked" , 1 );
    
    //let a rip
    ros::spin();
    
    
    return 0;
}

