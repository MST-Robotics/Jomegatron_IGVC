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


/***********************************************************
* @fn edgesCallback(const sensor_msgs::ImageConstPtr& msg)
* @brief adds the edges image to map using a weight 
* @pre takes in a ros message of a raw or cv image
* @post image added to the map
* @param takes in a ros message of a raw or cv image 
***********************************************************/
void homographyCallback( const sensor_msgs::ImageConstPtr& image_sub_msg)
{
    //ROS_INFO("Pot_Nav: Statistics image receieved");

    cv_bridge::CvImagePtr cv_ptr_src; // source image pointer
    cv_bridge::CvImage           dst; // destination image
    cv::Mat                        T; // transform matrix

    //camera model
    image_geometry::PinholeCameraModel cam_model;

    //tranform listener
    tf::TransformListener tf_listener;

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

    //find the transform matrix
    T = find_perspective(params.theta_x, params.theta_y, params.theta_z, 
                         params.center_x, params.center_y);
    
    
    //setup dst
    dst.encoding = "rgb8";
    
    //transform image
    warpPerspective(cv_ptr_src->image, dst.image, T, cv_ptr_src->image.size() * (params.scale));
    
    image_pub_homography.publish(dst.toImageMsg());
    
    
    
    
    
    
    
    
    
    
    
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
    
    
    std::string topic = n.resolveName("image_rect");
    
    //check to see if user has defined an image to subscribe to 
    if (topic == "/image_rect") 
    {
        ROS_WARN("homography: image has not been remapped! Typical command-line usage:\n"
                "\t$ ./Edge_Detection image_rect:=<image topic> [transport]");
    }
    

    //create subsctiptions
    image_sub_cam = it.subscribe( n.resolveName("image_rect"), 1, homographyCallback);
    

    //create publishers
    laser_pub = n.advertise<sensor_msgs::LaserScan>( "homography_scan" , 5 );
    
    image_pub_homography = it.advertise( "homography_image" , 1 );
    
    //let a rip
    ros::spin();
    
    
    return 0;
}

