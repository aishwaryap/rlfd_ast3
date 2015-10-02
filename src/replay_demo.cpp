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

#define VEL_THRESH 5
#define POSE_DIFF_THRESH 0.0001
#define abs(x) x>0?x:-x
#define NEAR_ZERO_VEL 0.0001

std::ofstream angles_file, torques_file, tool_file, finger_file;

sensor_msgs::JointState current_state;
sensor_msgs::JointState current_effort;
jaco_msgs::FingerPosition current_finger;

geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped prev_pose;

bool heardPose = false;
bool heardJoinstState = false;
bool stopRecordingDemo = false;
bool recording = false;

ros::Publisher pub_velocity;

std::vector<geometry_msgs::Pose> poses;
std::vector<geometry_msgs::TwistStamped> cartesian_velocities;
	
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
	prev_pose = current_pose;
  current_pose = msg;
  if (recording) {
		poses.push_back(current_pose.pose);
  }
  
  heardPose = true;
  std::cout << "Heard pose\n";
  // tool_file << "Recorded " << current_pose << "\n";
}

//Joint state cb
void fingers_cb (const jaco_msgs::FingerPosition msg) {
  current_finger = msg;
  finger_file << current_finger << "\n";
}

// Keyboard interrupt cb 
void keyboard_interrupt_cb(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Going to stop recording demo...\n");
  stopRecordingDemo = true;
}

bool poseMatch(int i){
	if (abs(current_pose.pose.position.x - poses[i].position.x) > POSE_DIFF_THRESH)
		return false;
	if (abs(current_pose.pose.position.y - poses[i].position.y) > POSE_DIFF_THRESH)
		return false;
	if (abs(current_pose.pose.position.z - poses[i].position.z) > POSE_DIFF_THRESH)
		return false;
	if (abs(current_pose.pose.orientation.x - poses[i].orientation.x) > POSE_DIFF_THRESH)
		return false;
	if (abs(current_pose.pose.orientation.y - poses[i].orientation.y) > POSE_DIFF_THRESH)
		return false;
	if (abs(current_pose.pose.orientation.z - poses[i].orientation.z) > POSE_DIFF_THRESH)
		return false;
	return true;
}

bool zeroVelocity(geometry_msgs::TwistStamped velocityMsg) {
	double l_1 = abs(velocityMsg.twist.linear.x) + abs(velocityMsg.twist.linear.y)
			+ abs(velocityMsg.twist.linear.z) + abs(velocityMsg.twist.angular.x)
			+ abs(velocityMsg.twist.angular.y) + abs(velocityMsg.twist.angular.z);
	if (l_1 < NEAR_ZERO_VEL)
		return true;
	return false;
}


bool overshot(int i) {
	double diff1, diff2;
	diff1 = poses[i-1].position.x - poses[i].position.x;
	diff2 = current_pose.pose.position.x - poses[i].position.x;
	if (diff1 * diff2 < 0)
		return true;
	diff1 = poses[i-1].position.y - poses[i].position.y;
	diff2 = current_pose.pose.position.y - poses[i].position.y;
	if (diff1 * diff2 < 0)
		return true;
	diff1 = poses[i-1].position.z - poses[i].position.z;
	diff2 = current_pose.pose.position.z - poses[i].position.z;
	if (diff1 * diff2 < 0)
		return true;
	diff1 = poses[i-1].orientation.x - poses[i].orientation.x;
	diff2 = current_pose.pose.orientation.x - poses[i].orientation.x;
	if (diff1 * diff2 < 0)
		return true;
	diff1 = poses[i-1].orientation.y - poses[i].orientation.y;
	diff2 = current_pose.pose.orientation.y - poses[i].orientation.y;
	if (diff1 * diff2 < 0)
		return true;
	diff1 = poses[i-1].orientation.z - poses[i].orientation.z;
	diff2 = current_pose.pose.orientation.z - poses[i].orientation.z;
	if (diff1 * diff2 < 0)
		return true;
	return false;
}

void replay_demo(int rateHertz) {
	ros::Rate r(rateHertz);
	for(int i = 0; i < cartesian_velocities.size(); i++) {
		std::cout << "\n\ni = " << i << "\n";
		if (!zeroVelocity(cartesian_velocities[i])) {
			std::cout << "Non zero velocities at " << i << "\n";
		}
		int num_replayed_same_vel = 0;
		do {
			// std::cout << "Publishing velocity " << cartesian_velocities[i] << "\n";
			pub_velocity.publish(cartesian_velocities[i]);
			r.sleep();
			ros::spinOnce();
			num_replayed_same_vel++;
		} while(!poseMatch(i+1) && !zeroVelocity(cartesian_velocities[i]) && !overshot(i+1) && num_replayed_same_vel < 3);
		if (poseMatch(i+1)) {
			std::cout << "Pose match at i = " << i << "\n";
		}
		if (overshot(i+1)) {
			std::cout << "Overshot at i = " << i << "\n";
		}
	}
}

// Blocking call for user input
void pressEnter(){
	std::cout << "Press the ENTER key to continue";
	while (std::cin.get() != '\n')
		std::cout << "Please press ENTER\n";
}

bool velocitiesSafe(geometry_msgs::TwistStamped velocityMsg) {
	if (velocityMsg.twist.linear.x > VEL_THRESH)
		return false;
	if (velocityMsg.twist.linear.y > VEL_THRESH)
		return false;
	if (velocityMsg.twist.linear.z > VEL_THRESH)
		return false;
	if (velocityMsg.twist.angular.x > VEL_THRESH)
		return false;
	if (velocityMsg.twist.angular.y > VEL_THRESH)
		return false;
	if (velocityMsg.twist.angular.z > VEL_THRESH)
		return false;
	return true;
}


void halveAndAdd(geometry_msgs::TwistStamped velocityMsg) {
	geometry_msgs::TwistStamped newVelocityMsg;
	newVelocityMsg.twist.linear.x = velocityMsg.twist.linear.x / 2.0;
	newVelocityMsg.twist.linear.y = velocityMsg.twist.linear.y / 2.0;
	newVelocityMsg.twist.linear.z = velocityMsg.twist.linear.z / 2.0;
	newVelocityMsg.twist.angular.x = velocityMsg.twist.angular.x / 2.0;
	newVelocityMsg.twist.angular.y = velocityMsg.twist.angular.y / 2.0;
	newVelocityMsg.twist.angular.z = velocityMsg.twist.angular.z / 2.0;
	cartesian_velocities.push_back(newVelocityMsg);
}


void findCartesianVelocities(int listenRateHertz) {
	for (int i=1; i<poses.size(); i++) {
		geometry_msgs::TwistStamped velocityMsg;
		velocityMsg.twist.linear.x = (poses[i].position.x - poses[i-1].position.x) * listenRateHertz;
		velocityMsg.twist.linear.y = (poses[i].position.y - poses[i-1].position.y) * listenRateHertz;
		velocityMsg.twist.linear.z = (poses[i].position.z - poses[i-1].position.z) * listenRateHertz;
		velocityMsg.twist.angular.x = (poses[i].orientation.x - poses[i-1].orientation.x) * listenRateHertz;
		velocityMsg.twist.angular.y = (poses[i].orientation.y - poses[i-1].orientation.y) * listenRateHertz;
		velocityMsg.twist.angular.z = (poses[i].orientation.z - poses[i-1].orientation.z) * listenRateHertz;
		
		if (velocitiesSafe(velocityMsg)) {
			cartesian_velocities.push_back(velocityMsg);
		} else {
			halveAndAdd(velocityMsg);
		}
	}
}

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "subscriber");
	
	ros::NodeHandle n;
	
	// Files to store the recorded data
	angles_file.open("/home/bwi/aishwarya/joint_angles.txt");
	torques_file.open("/home/bwi/aishwarya/joint_torques.txt");
	tool_file.open("/home/bwi/aishwarya/tool_position.txt");
	finger_file.open("/home/bwi/aishwarya/finger_position.txt");

	//create subscriber to joint angles
	// ros::Subscriber sub_angles = n.subscribe ("/joint_states", 1, joint_state_cb);

	//create subscriber to joint torques
	// ros::Subscriber sub_torques = n.subscribe ("/mico_arm_driver/out/joint_efforts", 1, joint_effort_cb);

	//create subscriber to tool position topic - Cartesian end-effector position
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);
	
	// Subscribe to the code that will publish a keyboard interrupt when the demo is over
	ros::Subscriber sub = n.subscribe("keyboard_interrupt", 1000, keyboard_interrupt_cb);

	//publish velocities
	pub_velocity = n.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);

	//subscriber for fingers
	// ros::Subscriber sub_finger = n.subscribe("/mico_arm_driver/out/finger_position", 1, fingers_cb);

	// Logic for replaying demo
	//		First check what data you get in tool_position messages
	//		If the have velocities, just feed them back using the 
	//		function from copy.cpp else you know the rate at which you 
	//		get cartesian poses. Divide by time (got from frame rate)
	//		to get velocities

	std::cout << "Move robot to start position and press Enter to start recording";
	std::cin.get();

	recording = true;
	int listenRateHertz = 50;
	ros::Rate r(listenRateHertz);
	while (ros::ok() && !stopRecordingDemo) {
		ros::spinOnce();
	}
	recording = false;

	std::cout << "Stopped recording demo.\n";
	std::cout << "Recorded " << poses.size() << " frames \n";

	angles_file.close();
	torques_file.close();
	tool_file.close();
	finger_file.close();	
	
	// Calculate velocities
	findCartesianVelocities(listenRateHertz);
	
	std::cout << "Calculated Cartesian velocities. There are " << cartesian_velocities.size() << " velocities. \n";
	
	std::cout << "Move robot to start position and press enter to replay demo : ";
	std::cin.get();
	
	replay_demo(listenRateHertz);
	
	std::cout << "Replayed demo...";
}
