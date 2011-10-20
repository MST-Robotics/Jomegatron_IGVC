#include "ros/ros.h"
#include "joe_controller_pkg/ApplyJointEffort.h"
#include "joe_controller_pkg/CommandVelocity.h"
#include "joe_controller_pkg/GetJointProperties.h"
#include <cmath>



/*****************************************************************************************************************
change to function off of get_joint_properties services instead!
*****************************************************************************************************************/
double rotation_left, rotation_right, pivot_angle;
static double WHEEL_RADIUS = 0.05;
static double WHEEL_SEPARATION = 0.24;
static double PIVOT_WHEEL_SEPARATION = 0.157;

bool getRotation(joe_controller_pkg::CommandVelocity::Request &req, joe_controller_pkg::CommandVelocity::Response &resp)
{
	double linearVelocity = req.linear, angularVelocity = req.angular;
	rotation_left = linearVelocity / WHEEL_RADIUS - angularVelocity * (WHEEL_SEPARATION / 2) / WHEEL_RADIUS;
	rotation_right = linearVelocity / WHEEL_RADIUS + angularVelocity * (WHEEL_SEPARATION / 2) / WHEEL_RADIUS;
	pivot_angle = atan(PIVOT_WHEEL_SEPARATION * angularVelocity / linearVelocity);
	resp.rotation_left = rotation_left;
	resp.rotation_right = rotation_right;
	resp.pivot_angle = pivot_angle;
	return true;
}

int main(int argc, char **argv)
{
	rotation_left = 0;
	rotation_right = 0;
	pivot_angle = 0;

	ros::init(argc, argv, "joe_controller");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("/joe_controller_pkg/joe_control", getRotation);
	ros::ServiceClient effortClient = n.serviceClient<joe_controller_pkg::ApplyJointEffort>("/gazebo/apply_joint_effort");
	ros::ServiceClient jointClient = n.serviceClient<joe_controller_pkg::GetJointProperties>("/gazebo/get_joint_properties");

	joe_controller_pkg::GetJointProperties left_wheel, right_wheel, pivot_joint;
	joe_controller_pkg::ApplyJointEffort effort_left, effort_right, effort_pivot;
	effort_left.request.joint_name = "left_wheel_axis";
	effort_right.request.joint_name = "right_wheel_axis";
	effort_pivot.request.joint_name = "pivot_joint";
	left_wheel.request.joint_name = "left_wheel_axis";
	right_wheel.request.joint_name = "right_wheel_axis";
	pivot_joint.request.joint_name = "pivot_joint";

	while(ros::ok())
	{	
		ros::spinOnce();

		//get the states for each of the links
		if(jointClient.call(pivot_joint))
			ROS_INFO("Successfully called get_joint_properties for pivot_link");
		else
			ROS_ERROR("Failed to call service get_joint_properties for pivot_link");
		if(jointClient.call(left_wheel))
			ROS_INFO("Successfully called get_joint_properties for left_wheel");
		else
			ROS_ERROR("Failed to call service get_joint_properties for right_wheel");
		if(jointClient.call(right_wheel))
			ROS_INFO("Successfully called get_joint_properties for left_wheel");
		else
			ROS_ERROR("Failed to call service get_joint_properties for left_wheel");


		/*set torques based on difference between expected and actual wheel orientations and velocities*/
		if(pivot_joint.response.position[0] > pivot_angle)
			effort_pivot.request.effort = -1000.0;
		else if(pivot_joint.response.position[0] < pivot_angle)
			effort_pivot.request.effort = 1000.0;
		if(right_wheel.response.rate[0] > rotation_right)
			effort_right.request.effort = -1000.0;
		else if(right_wheel.response.rate[0] < rotation_right)
			effort_right.request.effort = 1000.0;
		if(left_wheel.response.rate[0] > rotation_left)
			effort_left.request.effort = -1000.0;
		else if(left_wheel.response.rate[0] < rotation_left)
			effort_left.request.effort = 1000.0;


		/*call apply_joint_effort to apply torques to the wheels and pivot joint */
		if(effortClient.call(effort_pivot))
			ROS_INFO("Successfully called apply_joint_effort to pivot_joint");
		else
			ROS_ERROR("Failed to call apply_joint_effort to pivot_joint");
		if(effortClient.call(effort_left))
			ROS_INFO("Successfully called apply_joint_effort to left_wheel");
		else
			ROS_ERROR("Failed to call apply_joint_effort to left_wheel");
		if(effortClient.call(effort_right))
			ROS_INFO("Successfully called apply_joint_effort to right_wheel");
		else
			ROS_ERROR("Failed to call apply_joint_effort to right_wheel");
	}
	return 0;
}
