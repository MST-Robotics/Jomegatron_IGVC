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
void homographyCallback( const sensor_msgs::ImageConstPtr& image_msg,
                         const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    //ROS_INFO("Pot_Nav: Statistics image receieved");

    cv_bridge::CvImagePtr cv_ptr_src;
    std::vector<cv::Mat> Chanels;

    //takes in the image
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
    theta_x = acos(transform.getRotation().x()/sin(alph/2))
    theta_y = acos(transform.getRotation().y()/sin(alph/2))
    theta_z = acos(transform.getRotation().z()/sin(alph/2))
    
    //find the transform matrix
    Mat T = find_perspective(theta_x, theta_y, theta_z);
    
    T
    
    /* This is the start of a hacky way where 4 imaginary points on the ground 
    plain are used to find the transform
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
* @fn targetCallback( const MST_Position::Target_Heading::ConstPtr& msg)
* @brief copys taget message to global variable 
***********************************************************/
void targetCallback( const MST_Position::Target_Heading::ConstPtr& msg)
{
	target.target_heading = msg->target_heading;
	target.waypoint = msg->waypoint;
	target.distance = msg->distance;
	target.stop_robot = msg->stop_robot;
	target.done = msg->done;
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
* @fn find_twist()
* @brief computes the twists using the map
* @pre needs a globaly defined map it uses to compute twist
* @post computes the twist
* @return reterns the gemoetry messages standard twist and 
* draws debug info on map_dis
***********************************************************/
geometry_msgs::Twist find_twist()
{
	cv::Point2i robot_center;
    geometry_msgs::Twist twist;
    cv::Point2i box_1;
    cv::Point2i box_2;
    
    
    //find botom center of the image
    robot_center.x = map.image.cols/2 ;
    robot_center.y = map.image.rows - 1 ;
    
    box_1.x = robot_center.x - params.robot_x;
    box_1.y = robot_center.y - params.robot_y;
    
    box_2.x = robot_center.x + params.robot_x;
    box_2.y = robot_center.y + params.robot_y;
    
    map.image.copyTo(map_dis.image);
    map_dis.header = map.header;
    map_dis.encoding = map.encoding;
    
    //draw box in place of robot
	cv::rectangle(map_dis.image, box_1, box_2 , 255 ,-2*params.robot_filled+1);
	
	//remove robot from the map
	cv::rectangle(map.image, box_1, box_2 , 0,-1);
	
	//draw the search radius
	cv::circle(map_dis.image, robot_center, params.search_radius, 255);
	
    //initalize twist and apply forward foring funciton
	twist.linear.y = 0;
	twist.linear.x = params.carrot_on_a_stick;
    twist.linear.z = 0;
    
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    
    //add  in target
    twist.linear.x =  twist.linear.x + (params.target_weight_y/100.0) * sin(target.target_heading)/ (target.distance * params.target_dist_scale/1000) ;
    twist.angular.z = twist.angular.z + (params.target_weight_z/100.0) * cos(target.target_heading)/ (target.distance * params.target_dist_scale/1000);
    
	ROS_INFO("y: %f x: %f" , sin(target.target_heading) , cos(target.target_heading));
    
    //compute twist
    for(int deg = 0 ;  deg <= params.search_res ; deg++ )
    {
		
		float degree = deg;
		float res = params.search_res;
		float theta = pi*(degree/res);
		cv::Point2f search_edge;
		
		
		//compute the edge point
		search_edge.x = (robot_center.x + ((cos(theta) * params.search_radius)));
		search_edge.y = (robot_center.y - ((sin(theta) * params.search_radius)));
		
		//draw rays
		/*
		if(params.display_rays)
		{
			line(map_dis.image, robot_center, search_edge , 255);
		}
		
		*/
		
		cv::LineIterator ray(map.image, robot_center, search_edge , 8);
		
		for(int pos = 0 ; pos < ray.count ; pos++ , ++ray)
		{

			    
			if(ray.pos().x > robot_center.x + params.robot_x || 
			   ray.pos().x < robot_center.x - params.robot_x ||
			   ray.pos().y < robot_center.y - params.robot_y )
			{
			
			
				/*
				@todo This is most definatly the wrong way to access pixls on the iterator sice it should return a pointer
				but I give up for now
				*/
				
				
				if(params.display_rays)
				{
					map_dis.image.at<float>( ray.pos().y , ray.pos().x ) = 200; 
				}
				
				
			    //magnitued divided by the distance squared

				 
			    double dist = (params.dist_scale_x/1000 * pow(abs(ray.pos().x - robot_center.x ),2) 									+  params.dist_scale_y/1000 * pow(abs(ray.pos().y -robot_center.y ),2)) ;


			    float mag = map.image.at<float>( ray.pos().y , ray.pos().x );
                
        		

			    
				twist.linear.x -= (params.twist_scalar_y/1000  * mag/dist * sin(theta));
				twist.angular.z += (params.twist_scalar_z/1000  * mag/dist * cos(theta));	   

 			} 
 			//twist.linear.x = twist.linear.x*100/params.search_res ;
 			//twist.angular.z = twist.angular.z*100/params.search_res ;
 			
		}
		

		
		
		
	}
       
	cv::Point2i line;

	line.x = robot_center.x - params.compas_length * twist.angular.z;
	line.y = robot_center.y - params.compas_length * twist.linear.x;
			
	//draw movement
	cv::line(map_dis.image, robot_center, line , 255 , 5);
	
	//draw target
	line.y = robot_center.y - params.compas_length * sin(target.target_heading);
	line.x = robot_center.x - params.compas_length * cos(target.target_heading);
	
	
	cv::line(map_dis.image, robot_center,line,255 , 3);   
    
    return twist; 
}

/***********************************************************
* @fn stop_robot()
* @brief sends a velocity of zero
* @post returns a twist of zero
***********************************************************/
geometry_msgs::Twist stop_robot()
{
	geometry_msgs::Twist twist;
	
	twist.linear.x = 0;
	twist.linear.y = 0;
    twist.linear.z = 0;
    
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    
    return twist;
}

/*
void check_queue()
{

	if(stat_time_q.size() != 0 && edges_time_q.size() != 0)
	{
		if(stat_time_q.front() == edges_time_q.front())
		{
			map_changed = 1;
			//ROS_INFO("2");
		}
		else if(stat_time_q.front() > edges_time_q.front())
		{
			edges_time_q.pop();
			edges_q.pop();
			check_queue();
			//ROS_INFO("3");
		}
		else if(stat_time_q.front() < edges_time_q.front())
		{
			stat_time_q.pop();
			stat_q.pop();
			check_queue();
			//ROS_INFO("4");
		}
	}
	
}
*/

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

    //create subsctiptions
    
    image_sub_stat = it.subscribe( n.resolveName("image_stat") , 10, statCallback );
    
    target_sub = n.subscribe( "/target" , 5, targetCallback );

    //create publishers
    twist_pub = n.advertise<geometry_msgs::Twist>( "nav_twist" , 5 );
    map_pub = it.advertise( "map" , 5 );
    
    //set rate to 30 hz
    ros::Rate loop_rate(60);
    
    //run main loop
	while (ros::ok())
    {
		//check calbacks
		ros::spinOnce();
		
		//finds and publishes new twist
		//check_queue();
		if(map_changed )
		{
			geometry_msgs::Twist twist;
			
			/*
			map.image = map.image * params.previous_per/100 + stat_q.front() + edges_q.front() ;
			
			stat_time_q.pop();
			stat_q.pop();
			edges_time_q.pop();
			edges_q.pop();
			*/
			
			map.image = map.image * params.previous_per/100 + stat;
			
			if(target.stop_robot)
			{
				twist = find_twist(); 
				twist = stop_robot();
			}
			else
			{
				twist = find_twist(); 
			}
			
			//publish twist
			twist_pub.publish(twist);
			
			//publish map
			map_pub.publish(map_dis.toImageMsg());
			
			map_changed = 0;
		}
		
		loop_rate.sleep();
    }
    
    return 0;
}

