#include <ros/ros.h>

#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>
#include <fstream>
#include <sys/time.h>
#include <cmath>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>

#include "jaco_msgs/SetFingersPositionAction.h"
#include "jaco_msgs/ArmPoseAction.h"
#include "jaco_msgs/ArmJointAnglesAction.h"

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#define NUM_JOINTS 8 //6+2 for the arm

#define VEL_THRESH 0.1
#define POSE_DIFF_THRESH 0.0001
#define abs(x) x>0?x:-x
#define NEAR_ZERO_VEL 0.0001
#define TRAJ_TIME_STEP 1

geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped prev_pose;

bool heardPose = false;
bool stopRecordingDemo = false;
bool recording = false;

ros::Publisher pub_velocity;

// Vectors to be used by the DMP
std::vector< int > t;
std::vector< double > s;
std::vector< std::vector<double> > x;
std::vector< std::vector<double> > v;
std::vector< std::vector<double> > a;
std::vector< std::vector<double> > f;

std::vector< std::vector<double> > traj_x;
std::vector< std::vector<double> > traj_v;
std::vector< std::vector<double> > traj_a;
std::vector< std::vector<double> > traj_f;


timeval start_of_demo, current_pose_timestamp, prev_pose_timestamp;

double alpha = 2.0 * log(10);
double tau = 0;
double K = 10;
double D = 2.0 * sqrt(K);

std::vector<double> x_0;
std::vector<double> g;

visualization_msgs::MarkerArray marker_array;

// This creates a marker point for rviz, given the position at which it should be
visualization_msgs::Marker create_marker(std::vector<double> pose) {
	static int id = 0;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/base_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "points";
	marker.action = visualization_msgs::Marker::ADD;
	
	marker.pose.position.x = pose[0];
	marker.pose.position.y = pose[1];
	marker.pose.position.z = pose[2];
	marker.pose.orientation.x = pose[3];
	marker.pose.orientation.y = pose[4];
	marker.pose.orientation.z = pose[5];
	marker.pose.orientation.w = pose[6];
	
	marker.id = id++;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0;
	return marker;
}

std::vector<double> get_pose_vector(geometry_msgs::PoseStamped pose) {
    std::vector<double> cur_x(7);
    cur_x[0] = pose.pose.position.x;
    cur_x[1] = pose.pose.position.y;
    cur_x[2] = pose.pose.position.z;
    cur_x[3] = pose.pose.orientation.x;
    cur_x[4] = pose.pose.orientation.y;
    cur_x[5] = pose.pose.orientation.z;
    cur_x[6] = pose.pose.orientation.w;
    return cur_x;
}

double calc_s(double t) {
    return exp(- alpha * t / tau);
}

void push_current_pose() {
    x.push_back(get_pose_vector(current_pose));
}

void set_x_0_and_g() {
    x_0 = x[0];
    g = x[x.size() - 1];
}

bool near_zero_vel(std::vector<double> velocity) {
	double magnitude = 0;
	for (int i=0; i<7; i++) {
		magnitude += (velocity[i] * velocity[i]);
	}
	magnitude = sqrt(magnitude);
	if (magnitude < NEAR_ZERO_VEL) {
		return true;
	}
	return false;
}

void learn_dmp() {
    v = std::vector< std::vector<double> >(t.size());
    a = std::vector< std::vector<double> >(t.size());
    f = std::vector< std::vector<double> >(t.size());
    s = std::vector<double>(t.size());
    
    std::cout << "Initialized vectors...\n";
    
    //std::cout << "t.size() = " << t.size() << "\n";
    //std::cout << "Now going to start loop\n";
   
    
    for (int i=0; i<t.size(); i++) {
		//std::cout << "i = " << i << ":\n";
        s[i] = calc_s(t[i]);
        //std::cout << "s[" << i << "] = " << s[i] << "\n";
        v[i] = std::vector<double>(7);
        //std::cout << "v[" << i << "].size() = " << v[i].size() << "\n";
        a[i] = std::vector<double>(7);
        //std::cout << "a[" << i << "].size() = " << a[i].size() << "\n";
        f[i] = std::vector<double>(7);
        //std::cout << "f[" << i << "].size() = " << f[i].size() << "\n";
        for (int j=0; j<7; j++) {
			//std::cout << "\tj = " << j << "\n";
            if (i < 1) {
                v[i][j] = 0;
                a[i][j] = 0;
            } else {
                v[i][j] = (x[i][j] - x[i-1][j]) / (t[i] - t[i-1]);
                a[i][j] = (v[i][j] - v[i-1][j]) / (t[i] - t[i-1]);
            }    
            //std::cout << "g.size() = " << g.size() << ", x_0.size() = " << x_0.size() << "\n";
            f[i][j] = ((tau * a[i][j]) - (D * v[i][j])) / K
                        + (g[j] - x_0[j]) * s[i]
                        + (g[j] - x[i][j]);
        }
        //std::cout << "\n";
    }
    std::cout << "Learnt DMP...\n";
}

int get_time_diff_ms(timeval tv) {
    return (((tv.tv_sec - start_of_demo.tv_sec) * 1000000) + 
            (tv.tv_usec - start_of_demo.tv_usec)/1000 );
}

int get_last_pose_time_ms() {
    return get_time_diff_ms(current_pose_timestamp);
}

int get_cur_time_ms() {
    timeval cur_time;
    gettimeofday(&cur_time, NULL);   
    return get_time_diff_ms(cur_time);
}

void push_current_time() {
    t.push_back(get_last_pose_time_ms());
}

//Joint state cb
void toolpos_cb (const geometry_msgs::PoseStamped &msg) {
    prev_pose_timestamp = current_pose_timestamp;
    gettimeofday(&current_pose_timestamp, NULL);   
    prev_pose = current_pose;
    current_pose = msg;
    if (recording) {
        push_current_time();
        push_current_pose();
    }
    heardPose = true;
    //std::cout << "Heard pose\n";
}

// Keyboard interrupt cb 
void keyboard_interrupt_cb(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Going to stop recording demo...\n");
  stopRecordingDemo = true;
}

bool reached_goal(){
	std::cout << "In reached goal \n";
    std::vector<double> cur_pose = get_pose_vector(current_pose);
    std::cout << "In reached goal before loop...\n";
    for (int i=0; i<7; i++) {
        if (abs(cur_pose[i] - g[i]) > POSE_DIFF_THRESH) {
			std::cout << "Mismatch at index " << i << "\n";
            return false;        
		}
    }
    std::cout << "Going to return true\n";
	return true;
}

std::vector<double> calc_f(double cur_s) {
    if (cur_s <= s[0]) {
        return f[0];
    } else if (cur_s >= s[s.size() - 1]) {
        return f[f.size() - 1];
    } else {
        for (int i=0; i<s.size(); i++) {
            if (s[i] == cur_s) {
                return f[i];
            } else if (s[i] > cur_s) {
                std::vector<double> cur_f(7);
                for (int j=0; j<7; j++) {
                    cur_f[j] = f[i-1][j] + ((f[i][j] - f[i-1][j]) / 
                        (s[i] - s[i-1]) * (cur_s - s[i-1]));
                }
                return cur_f;
            }
        }
    }
    
    // You should have returned some f by now else there is something wrong
    std::cout << "Error in f interpolation!\n";
    exit(1);
}

geometry_msgs::TwistStamped get_velocity_msg(std::vector<double> velocities) {
    geometry_msgs::TwistStamped velocityMsg;
    velocityMsg.twist.linear.x = velocities[0];
    velocityMsg.twist.linear.y = velocities[1];
    velocityMsg.twist.linear.z = velocities[2];
    velocityMsg.twist.angular.x = velocities[3];
    velocityMsg.twist.angular.y = velocities[4];
    velocityMsg.twist.angular.z = velocities[5];
    return velocityMsg;
}

std::vector<double> make_velocities_safe(std::vector<double> velocities) {
    bool safe = false;
    while (!safe) {
        safe = true;
        for (int i=0; i<7; i++) {
            if (abs(velocities[i]) > VEL_THRESH) {
                safe = false;
                break;
            }
        }
        if (!safe) {
            for (int i=0; i<7; i++) {
                velocities[i] /= 2;
            }
        }
    }
    std::cout << "Returning velocities: ";
    for (int i=0; i<7; i++) {
		std::cout << velocities[i] << " ";
	}
	std::cout << "\n";
    return velocities;
}

std::vector<double> normalize_quaternion(std::vector<double> quaternion) {
	double magnitude = quaternion[3] + quaternion[4] + quaternion[5] + quaternion[6];
    std::vector<double> normalized_quaternion = std::vector<double>(quaternion);
    for (int i=3; i<7; i++) {
        normalized_quaternion[i] /= magnitude ;
    }
    return normalized_quaternion;
}

void replay_demo(int rateHertz) {
	ros::Rate r(rateHertz);
    gettimeofday(&start_of_demo, NULL); 
    std::cout << "Here...\n";
    while (!reached_goal()) {
		std::cout << "Start of replay loop...\n";
        std::vector<double> cur_x = get_pose_vector(current_pose);
        std::vector<double> prev_x = get_pose_vector(prev_pose);        
        std::vector<double> cur_v(7);
        std::cout << "Calculated current velocity...\n";
        
        double cur_t = get_time_diff_ms(current_pose_timestamp);
        double cur_s = calc_s(cur_t);
        
        std::cout << "Before for loop...\n";
        for (int i=0; i<7; i++) {
            cur_v[i] = (cur_x[i] - prev_x[i]) 
                / (cur_t - get_time_diff_ms(prev_pose_timestamp));
        }
        
        std::vector<double> cur_f = calc_f(cur_s);
        std::vector<double> desired_a(7), desired_v(7);
        std::cout << "Before next for loop...\n";
        for (int i=0; i<7; i++) {
            desired_a[i] = (K * (g[i] - cur_x[i]) - D * cur_v[i] - 
                K * (g[i] - x_0[i]) * cur_s + K * cur_f[i]) / tau;
            desired_v[i] = cur_v[i] + desired_a[i] * (1000 / rateHertz);
        }
        
        std::cout << "Before normalization...\n";
        desired_v = normalize_quaternion(desired_v);
        //std::cout << "Before safety...\n";
        //desired_v = make_velocities_safe(desired_v);
        
        std::cout << "Going to publish velocities: ";
		for (int i=0; i<7; i++) {
			std::cout << desired_v[i] << " ";
		}
		std::cout << "\n";
        
        if (!near_zero_vel(desired_v)) {
			std::cout << "Converting to message...\n";
			geometry_msgs::TwistStamped velocity_msg = get_velocity_msg(desired_v);
			std::cout << "Going to publish...\n";
			pub_velocity.publish(velocity_msg);
			std::cout << "Published " << velocity_msg << "\n";
			r.sleep();
			std::cout << "Came out of sleep\n";
			ros::spinOnce();
			std::cout << "End of loop\n";
		} else {
			std::cout << "Not publishing zero velocity...\n";
		}
    }
}

double magnitude_diff(std::vector<double> v1, std::vector<double> v2){
	double diff = 0;
	for (int i=0; i<7; i++) {
		diff += (v1[i] - v2[i]) * (v1[i] - v2[i]);
	}
	return sqrt(diff);
}

std::vector<double> get_zero_vector(int size) {
	std::vector<double> zero(size);
	for (int i=0; i<size; i++) {
		zero[i] = 0;
	}
	return zero;
}

void replay_calculated_trajectory(int rateHertz) {
	ros::Rate r(rateHertz);
	
	int freq_to_use = (1000/rateHertz) / TRAJ_TIME_STEP;
	
	for (int i=0; i<traj_v.size(); i+=freq_to_use) {
		std::cout << "Going to publish velocities: ";
		for (int j=0; j<7; j++) {
			std::cout << traj_v[i][j] << " ";
		}
		std::cout << "\n";
        
        if (!near_zero_vel(traj_v[i])) {
			std::cout << "Converting to message...\n";
			geometry_msgs::TwistStamped velocity_msg = get_velocity_msg(traj_v[i]);
			std::cout << "Going to publish...\n";
			pub_velocity.publish(velocity_msg);
			std::cout << "Published " << velocity_msg << "\n";
			r.sleep();
			std::cout << "Came out of sleep\n";
			ros::spinOnce();
			std::cout << "End of loop\n";
		} else {
			std::cout << "Not publishing zero velocity...\n";
		}
	}
}



void calculate_trajectory() {
	std::vector<double> cur_x = x_0, prev_x, cur_v, cur_f, cur_a, next_v;
	double cur_t, cur_s;
	bool first_point = true;
	
	int cc =0;
	while (magnitude_diff(cur_x, g) > POSE_DIFF_THRESH) {
		std::cout << "Distance to go: " << magnitude_diff(cur_x, g) << "\n";
		cc++;
		if (cc == 200){
			break;
		}
		
	    if (first_point) {
			first_point = false;
			prev_x = cur_x;
			cur_v = get_zero_vector(7);
			cur_t = 0;
		}
	    cur_s = calc_s(cur_t);
        cur_f = calc_f(cur_s);
        
        cur_a = get_zero_vector(7);
        next_v = get_zero_vector(7);
        for (int i=0; i<7; i++) {
            cur_a[i] = (K * (g[i] - cur_x[i]) - D * cur_v[i] - 
                K * (g[i] - x_0[i]) * cur_s + K * cur_f[i]) / tau;
            next_v[i] = cur_v[i] + cur_a[i] * TRAJ_TIME_STEP;
        }
       
        next_v = normalize_quaternion(next_v);
        //std::cout << "Before safety...\n";
        //desired_v = make_velocities_safe(desired_v);

		std::cout << "next_v: ";
		for (int i=0; i<7; i++) {
			std::cout << next_v[i] << " ";
		}
		std::cout << "\n";

		std::cout << "cur_x: ";
		for (int i=0; i<7; i++) {
			std::cout << cur_x[i] << " ";
		}
		std::cout << "\n";

		// Adding a point to the rviz marker
		visualization_msgs::Marker marker = create_marker(cur_x);
		marker_array.markers.push_back(marker);
        
        traj_x.push_back(cur_x);
        traj_v.push_back(cur_v);
        traj_a.push_back(cur_a);
        traj_f.push_back(cur_f);
        
        prev_x = cur_x;
        for (int i=0; i<7; i++) {
			cur_x[i] = next_v[i] * TRAJ_TIME_STEP;	
		}
        cur_v = next_v;
    }
}

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "subscriber");
	
	ros::NodeHandle n;
	
	//create subscriber to tool position topic - Cartesian end-effector position
	ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);
	
	// Subscribe to the code that will publish a keyboard interrupt when the demo is over
	ros::Subscriber sub = n.subscribe("keyboard_interrupt", 1000, keyboard_interrupt_cb);

	//publish velocities
	pub_velocity = n.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);

	std::cout << "Move robot to start position and press Enter to start recording";
	std::cin.get();
	std::cout << "Recording demo. Press enter in the keyboard interrupter to stop...\n";

	// Publisher to publish in RViz
	ros::Publisher marker_pub =  
		n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
	
	recording = true;
	int listenRateHertz = 50;
	ros::Rate r(listenRateHertz);
    gettimeofday(&start_of_demo, NULL); 
	while (ros::ok() && !stopRecordingDemo) {
		ros::spinOnce();
	}
    tau = get_last_pose_time_ms();
	recording = false;

	std::cout << "Stopped recording demo.\n";
	std::cout << "Recorded " << x.size() << " frames\n";
    
    if (x.size() != t.size()) {
        std::cout << "x.size() = " << x.size() << ", t.size() = " << t.size() << "\n";
        exit(1);
    } 
    
    set_x_0_and_g();
	learn_dmp();
    
    calculate_trajectory();
    
    // Adjust colors of rviz markers so that they fade from green to red
	int num_points = traj_x.size();
	float g_amount = 1.0;
	float r_amount = 0.0;
	float per_point_color_diff = 1.0 / num_points;
	for (int point_num = 0; point_num <= num_points; point_num++) {
		marker_array.markers[point_num].color.g = g_amount;
		marker_array.markers[point_num].color.r = r_amount;
		g_amount -= per_point_color_diff;
		r_amount += per_point_color_diff;
	}
	// Display points in rviz
	marker_pub.publish(marker_array);
    
    // Reset tau for desired duration of demo
    
	std::cout << "Move robot to start position and press enter to replay demo : ";
	std::cin.get();
	
	//replay_demo(listenRateHertz);
	replay_calculated_trajectory(listenRateHertz);
	
	std::cout << "Replayed demo...";
}
