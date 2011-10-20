#include "ros/ros.h"
#include "joe_controller_pkg/ApplyJointEffort.h"
#include "joe_controller_pkg/CommandVelocity.h"
#include "joe_controller_pkg/GetLinkState.h"

double rotation_left, rotation_right;
static double WHEEL_RADIUS = 0.05;
static double WHEEL_SEPARATION = 0.24;

/* getRotation gets Joe's velocity commands, and assigns a suitable rotation speed to each of the wheels*/
bool getRotation(joe_controller_pkg::CommandVelocity::Request &req, joe_controller_pkg::CommandVelocity::Response &resp)
{
	double linearVelocity = req.linear, angularVelocity = req.angular;
	rotation_left = linearVelocity / WHEEL_RADIUS - angularVelocity * (WHEEL_SEPARATION / 2) / WHEEL_RADIUS;
	rotation_right = linearVelocity / WHEEL_RADIUS + angularVelocity * (WHEEL_SEPARATION / 2) / WHEEL_RADIUS;
	resp.rotation_left = rotation_left;
	resp.rotation_right = rotation_right;
	return true;
}

/*Begin program*/
int main(int argc, char **argv)
{
	/*initially sets rotation speeds to zero, so Joe doesn't go anywhere unless commanded*/
	rotation_left = 0;
	rotation_right = 0;
	
	/*sets up the node and its service provider/clients*/
	ros::init(argc, argv, "joe_controller");
	ros::NodeHandle n;
	/*joe_control service provides service that accepts velocity commands, and returns the wheel rotation speeds*/
	ros::ServiceServer service = n.advertiseService("/joe_controller_pkg/joe_control", getRotation);
	/*apply_joint_effort service is called to apply torques to the wheels, this is used to push Joe*/
	ros::ServiceClient jointClient = n.serviceClient<joe_controller_pkg::ApplyJointEffort>("/gazebo/apply_joint_effort");
	/*get_link_state service is called to get the wheel's position, so torques can be properly applied*/
	ros::ServiceClient linkClient = n.serviceClient<joe_controller_pkg::GetLinkState>("/gazebo/get_link_state");

	/*initialize service messages and assign them to the left and right wheels*/
	joe_controller_pkg::GetLinkState left_wheel, right_wheel;
	joe_controller_pkg::ApplyJointEffort effort_left, effort_right;
	effort_left.request.joint_name = "Jomegatron::left_axle";
	effort_right.request.joint_name = "Jomegatron::right_axle";
	//effort_left.request.start_time = 1000000000;
	//effort_right.request.start_time = 1000000000;
	//effort_left.request.duration = -1;
	//effort_right.request.duration = -1;
	left_wheel.request.link_name = "Jomegatron::left_wheel";
	right_wheel.request.link_name = "jomegatron::right_wheel";
	left_wheel.request.reference_frame = "world";
	right_wheel.request.reference_frame = "world";

	/*create time management variable*/
	double previous_time = ros::Time::now().toSec(), expected_position_left, expected_position_right;

	/*enter loop that calls and recieves commands as quickly as possible*/
	while(ros::ok())
	{
		/*calculates the expected positions of the wheels from the previous command to be compaired to their current positions
		so that suitable torques may be applied*/
		expected_position_left = left_wheel.response.link_state.pose.orientation.z + rotation_left * (ros::Time::now().toSec() - previous_time);
		expected_position_right = right_wheel.response.link_state.pose.orientation.z + rotation_right * (ros::Time::now().toSec() - previous_time);
		/*recieve the latest velocity command*/
		ros::spinOnce(); 

		/*call get_link_state_service to get current wheel orientations*/
		if(linkClient.call(left_wheel))
		{
			ROS_INFO("Successfully called get_link_state for left_wheel");
		}
		else
		{
			ROS_ERROR("Failed to call service get_link_state for right_wheel");
		}
		if(linkClient.call(right_wheel))
		{
			ROS_INFO("Successfully called get_link_state for left_wheel");
		}
		else
		{
			ROS_ERROR("Failed to call service get_link_state for left_wheel");
		}
		/*calculate torques based on difference between expected and actual wheel orientations*/
		effort_left.request.effort = 10 * (expected_position_left - left_wheel.response.link_state.pose.orientation.z);
		effort_right.request.effort = 10 * (expected_position_right - right_wheel.response.link_state.pose.orientation.z);

		/*call apply_joint_effort to apply torques to the wheels*/
		if(jointClient.call(effort_left))
		{
			ROS_INFO("Successfully called apply_joint_effort to left_wheel");
		}
		else
		{
			ROS_ERROR("Failed to call apply_joint_effort to left_wheel");
		}
		if(jointClient.call(effort_right))
		{
			ROS_INFO("Successfully called apply_joint_effort to right_wheel");
		}
		else
		{
			ROS_INFO("Failed to call apply_joint_effort to right_wheel");
		}
	}
	return 0;
	/*program ends*/
}

