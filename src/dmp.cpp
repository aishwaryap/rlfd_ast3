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

#define NUM_JOINTS 8 //6+2 for the arm

#define VEL_THRESH 5
#define POSE_DIFF_THRESH 0.0001
#define abs(x) x>0?x:-x
#define NEAR_ZERO_VEL 0.0001

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

timeval start_of_demo, current_pose_timestamp, prev_pose_timestamp;

double alpha = 2.0 * log(10);
double tau = 0;
double K = 10;
double D = 2.0 * sqrt(K);

std::vector<double> x_0;
std::vector<double> g;

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

void learn_dmp() {
    v = std::vector< std::vector<double> >(t.size());
    a = std::vector< std::vector<double> >(t.size());
    f = std::vector< std::vector<double> >(t.size());
    s = std::vector<double>(t.size());
    
    for (int i=0; i<t.size(); i++) {
        s[i] = calc_s(t[i]);
        v[i] = std::vector<double>(7);
        a[i] = std::vector<double>(7);
        f[i] = std::vector<double>(7);
        for (int j=0; j<7; j++) {
            if (i < 1) {
                v[i][j] = 0;
                a[i][j] = 0;
            } else {
                v[i][j] = (x[i][j] - x[i-1][j]) / (t[i] - t[i-1]);
                a[i][j] = (v[i][j] - v[i-1][j]) / (t[i] - t[i-1]);
            }    
            f[i][j] = ((tau * a[i][j]) - (D * v[i][j])) / K
                        + (g[j] - x_0[j]) * s[i]
                        + (g[j] - x[i][j]);
        }
    }
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
    std::cout << "Heard pose\n";
}

// Keyboard interrupt cb 
void keyboard_interrupt_cb(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Going to stop recording demo...\n");
  stopRecordingDemo = true;
}

bool reached_goal(){
    std::vector<double> cur_pose = get_pose_vector(current_pose);
    for (int i=0; i<7; i++) {
        if (abs(cur_pose[i] - g[i]) > POSE_DIFF_THRESH)
            return false;        
    }
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
}

std::vector<double> make_velocities_safe(std::vector<double> velocities) {
    bool safe = false;
    while (!safe) {
        safe = true;
        for (int i=0; i<7; i++) {
            if (velocities[i] > VEL_THRESH) {
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
    return velocities;
}

std::vector<double> normalize_quaternion(std::vector<double> quaternion) {
    double magnitude = quaternion[3] + quaternion[4] + quaternion[5] + quaternion[6];
    std::vector<double> normalized_quaternion = std::vector<double>(quaternion);
    for (int i=3; i<7; i++) {
        normalized_quaternion[i] /= magnitude ;
    }
}

void replay_demo(int rateHertz) {
	ros::Rate r(rateHertz);
    gettimeofday(&start_of_demo, NULL); 
    while (!reached_goal()) {
        std::vector<double> cur_x = get_pose_vector(current_pose);
        std::vector<double> prev_x = get_pose_vector(prev_pose);        
        std::vector<double> cur_v(7);
        double cur_t = get_time_diff_ms(current_pose_timestamp);
        double cur_s = calc_s(cur_t);
        for (int i=0; i<7; i++) {
            cur_v[i] = (cur_x[i] - prev_x[i]) 
                / (cur_t - get_time_diff_ms(prev_pose_timestamp));
        }
        std::vector<double> cur_f = calc_f(cur_s);
        std::vector<double> desired_a(7), desired_v(7);
        for (int i=0; i<7; i++) {
            desired_a[i] = (K * (g[i] - cur_x[i]) - D * cur_v[i] - 
                K * (g[i] - x_0[i]) * cur_s + K * cur_f[i]) / tau;
            desired_v[i] = cur_v[i] + desired_a[i] * (1000 / rateHertz);
        }
        desired_v = normalize_quaternion(desired_v);
        desired_v = make_velocities_safe(desired_v);
        geometry_msgs::TwistStamped velocity_msg = get_velocity_msg(desired_v);
        pub_velocity.publish(velocity_msg);
        r.sleep();
        ros::spinOnce();
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
    
	learn_dmp();
    
    // Reset tau for desired duration of demo
    
	std::cout << "Move robot to start position and press enter to replay demo : ";
	std::cin.get();
	
	replay_demo(listenRateHertz);
	
	std::cout << "Replayed demo...";
}
