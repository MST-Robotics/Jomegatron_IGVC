/*******************************************************************************
* @file Pot_Nav.cpp
* @author James Anderson <jra798>
* @version 1.0
* @brief findes the forward and angular velocity to move the robot at 
* given a fuzzy model
******************************************************************************/




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
void homographyCallback( const sensor_msgs::ImageConstPtr& image_sub_msg,
                         const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    //ROS_INFO("Pot_Nav: Statistics image receieved");

    cv_bridge::CvImagePtr cv_ptr_src; // source image pointer
    cv_bridge::CvImage           dst; // destination image
    cv::Mat                        T; // transform matrix

    //crete pointer to source image
    try
    {
      cv_ptr_src = cv_bridge::toCvCopy(msg, "rgb8");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    //get camera model from message
    cam_model_.fromCameraInfo(info_msg);
    
    //crate transform to ground frame
    tf::StampedTransform transform;
    
    //get the transform between the camera frame and target frame
    try {
    ros::Time acquisition_time = info_msg->header.stamp;
    ros::Duration timeout(1.0 / 30);
    tf_listener_.waitForTransform(cam_model_.tfFrame(), params.frame_id,
                                  acquisition_time, timeout);
    tf_listener_.lookupTransform(cam_model_.tfFrame(), params.frame_id,
                                 acquisition_time, transform);
    }
    catch (tf::TransformException& ex) {
        ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        return;
    }
    
    //find the roll pitch and yaw
    float alpha, theta_x, theta_y, theta_z;
    
    alpha = 2*acos(transform.getRotation().w());
    theta_x = acos(transform.getRotation().x()/sin(alph/2));
    theta_y = acos(transform.getRotation().y()/sin(alph/2));
    theta_z = acos(transform.getRotation().z()/sin(alph/2));
    
    //find the transform matrix
    cv::Mat T = find_perspective(theta_x, theta_y, theta_z);
    
    //setup dst
    dst.encoding = "rgb8";
    
    //transform image
    warpPerspective(cv_ptr_src->image, dst.image, T, cv_ptr_src->image.size());
    
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
        cam_model_.project3dToPixel(pt_cv, uv);
    }
    cv::getPerspectiveTransform()
    */

}

//this function finds the perspective transform matrix given the angle between the two frames
Mat find_perspective(float theta_x, float theta_y, float, theta_z))
{
    //these are the matricies for the transform they could be combined but 
    //I dont have my calculator or matlab and dont feel linke doing it by hand
    Mat A = (Mat <float>(3, 3) <<
     1, 0, 0,
     0, cos(theta_x), -sin(theta_x),
     0, sin(theta_x),  cos(theta_x));
    
    Mat B = (Mat <float>(3, 3) <<
     cos(theta_y), 0, sin(theta_y),
     0, 1, 0, 
    -sin(theta_y), 0, cos(theta_y));
    
    Mat B = (Mat <float>(3, 3) <<
     cos(theta_z), -sin(theta_z), 0,
     sin(theta_z),  cos(theta_z), 0, 
     0, 0, 1);
     
     Mat T = A*B*C;
     
     return T;
}



/***********************************************************
* @fn setparamsCallback(const sensor_msgs::ImageConstPtr& msg)
* @brief callback for the reconfigure gui
* @pre has to have the setup for the reconfigure gui
* @post changes the parameters
***********************************************************/
void setparamsCallback(mst_homography::mst_homography_ParamsConfig &config, uint32_t level)
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
    ros::init(argc, argv, "Pot_Nav");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    
    //setup dynamic reconfigure
    dynamic_reconfigure::Server<mst_homography::mst_homography_ParamsConfig> srv;
    dynamic_reconfigure::Server<mst_homography::mst_homography_ParamsConfig>::CallbackType f;
    f = boost::bind(&setparamsCallback, _1, _2);
    srv.setCallback(f);
    
    
    std::string topic = n.resolveName("image_rect")
    
    //check to see if user has defined an image to subscribe to 
    if (topic == "/image_rect") 
    {
        ROS_WARN("homography: image has not been remapped! Typical command-line usage:\n"
                "\t$ ./Edge_Detection image_rect:=<image topic> [transport]");
    }
    

    //create subsctiptions
    image_sub_cam = it.subscribeCamera( n.resolveName("image_rect"), 1, homographyCallback , this);
    

    //create publishers
    laser_pub = n.advertise<sensor_msgs::LaserScan>( "nav_twist" , 5 );
    
    //let a rip
    ros::spin();
    
    
    return 0;
}

