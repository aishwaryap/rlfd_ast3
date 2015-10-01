#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>
#include <fstream>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>

#include "jaco_msgs/SetFingersPositionAction.h"
#include "jaco_msgs/ArmPoseAction.h"
#include "jaco_msgs/ArmJointAnglesAction.h"

#define NUM_JOINTS 8 //6+2 for the arm

std::ofstream angles_file, torques_file, tool_file, finger_file;

sensor_msgs::JointState current_state;
sensor_msgs::JointState current_effort;
jaco_msgs::FingerPosition current_finger;
geometry_msgs::PoseStamped current_pose;
bool heardPose = false;
bool heardJoinstState = false;

ros::Publisher pub_velocity;

//Joint state cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
	
	if (input->position.size() == NUM_JOINTS){
		current_state = *input;
		heardJoinstState = true;
	}
  ROS_INFO_STREAM(current_state);
  angles_file << current_state << "\n";
}

//Joint state cb
void joint_effort_cb (const sensor_msgs::JointStateConstPtr& input) {
  current_effort = *input;
  ROS_INFO_STREAM(current_effort);
  torques_file << current_effort << "\n";
}

//Joint state cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
  current_pose = msg;
  heardPose = true;
  ROS_INFO_STREAM(current_pose);
  tool_file << current_pose << "\n";
}

//Joint state cb
void fingers_cb (const jaco_msgs::FingerPosition msg) {
  current_finger = msg;
  finger_file << current_finger << "\n";
}

// TODO: May have to change this data type based on in what form you 
// get velocities
void replay_demo(std::vector<geometry_msgs::TwistStamped> 
	cartesian_velocities, int rateHertz) {
	geometry_msgs::TwistStamped velocityMsg;
	
	ros::Rate r(rateHertz);
	for(int i = 0; i < cartesian_velocities.size(); i++) {
		velocityMsg.twist = cartesian_velocities[i].twist;
		//velocityMsg.twist.linear.x = l_x;
		//velocityMsg.twist.linear.y = l_y;
		//velocityMsg.twist.linear.z = l_z;
		
		//velocityMsg.twist.angular.x = a_x;
		//velocityMsg.twist.angular.y = a_y;
		//velocityMsg.twist.angular.z = a_z;
		//velocityMsg.twist.angular.w = a_w;
		
		pub_velocity.publish(velocityMsg);
		r.sleep();
	}
}

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "subscriber");
	
	ros::NodeHandle n;

	// Files to store the recorded data
	angles_file.open("joint_angles.txt");
	torques_file.open("joint_torques.txt");
	tool_file.open("tool_position.txt");
	finger_file.open("finger_position.txt");

	//create subscriber to joint angles
	ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);

	//create subscriber to joint torques
	ros::Subscriber sub_torques = n.subscribe ("/mico_arm_driver/out/joint_efforts", 1, joint_effort_cb);

	//create subscriber to tool position topic - Cartesian end-effector position
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);

	//subscriber for fingers
	ros::Subscriber sub_finger = n.subscribe("/mico_arm_driver/out/finger_position", 1, fingers_cb);

	// Logic for replaying demo
	//		First check what data you get in tool_position messages
	//		If the have velocities, just feed them back using the 
	//		function from copy.cpp else you know the rate at which you 
	//		get cartesian poses. Divide by time (got from frame rate)
	//		to get velocities

	angles_file.close();
	torques_file.close();
	tool_file.close();
	finger_file.close();	
}
