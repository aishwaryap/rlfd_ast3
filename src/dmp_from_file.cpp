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

#define VEL_THRESH 0.5
#define POSE_DIFF_THRESH 0.01
#define abs(x) x>0?x:-x
#define NEAR_ZERO_VEL 0.001
#define ROS_RATE 50
#define NEAR_ZERO 0.000001

geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped prev_pose;

bool heardPose = false;
bool stopRecordingDemo = false;
bool recording = false;

ros::Publisher pub_velocity;

// Vectors to be used by the DMP
std::vector< double > t;
std::vector< double > s;
std::vector< std::vector<double> > x;
std::vector< std::vector<double> > v;
std::vector< std::vector<double> > a;
std::vector< std::vector<double> > n;
std::vector< std::vector<double> > n_dot;
std::vector< std::vector<double> > f;

std::vector< std::vector<double> > traj_x;
std::vector< std::vector<double> > traj_v;
std::vector< std::vector<double> > traj_a;
std::vector< std::vector<double> > traj_f;
std::vector< std::vector<double> > traj_n;

timeval start_of_demo, current_pose_timestamp, prev_pose_timestamp;

double alpha = 2.0 * log(10);
double tau = 0;
double K = 100;
double D = 2.0 * sqrt(K);

std::vector<double> x_0;
std::vector<double> g;

visualization_msgs::MarkerArray marker_array;

void convert_time_to_sec() {
	for (int i=0; i<t.size();i++) {
		t[i] /= 1000;
	}
}

double translation_diff(std::vector<double> v1, std::vector<double> v2){
	double diff = 0;
	for (int i=0; i<3; i++) {
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

double magnitude_diff(std::vector<double> v1, std::vector<double> v2){
	double diff = 0;
	for (int i=0; i<7; i++) {
		diff += (v1[i] - v2[i]) * (v1[i] - v2[i]);
	}
	return sqrt(diff);
}

// This creates a marker point for rviz, given the position at which it should be
visualization_msgs::Marker create_marker(std::vector<double> pose) {
	static int id = 0;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "mico_link_base";
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

double calc_s(double t) {
    return exp(- alpha * t / tau);
}

void set_x_0_and_g() {
    x_0 = x[0];
    g = x[x.size() - 1];
}

bool near_zero_vel(std::vector<double> velocity) {
	double magnitude = 0;
	for (int i=0; i<3; i++) {
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
    n = std::vector< std::vector<double> >(t.size());
    n_dot = std::vector< std::vector<double> >(t.size());
    f = std::vector< std::vector<double> >(t.size());
    s = std::vector<double>(t.size());
    
    std::cout << "Initialized vectors...\n";
    
    //std::cout << "t.size() = " << t.size() << "\n";
    //std::cout << "Now going to start loop\n";
   
    
    for (int i=0; i<t.size(); i++) {
    	//std::cout << "i = " << i << ":\n";
        s[i] = calc_s(t[i]);
        std::cout << "s[" << i << "] = " << s[i] << "\n";

        v[i] = std::vector<double>(7);
        a[i] = std::vector<double>(7);
        n[i] = std::vector<double>(7);
        n_dot[i] = std::vector<double>(7);
        f[i] = std::vector<double>(7);

        //std::cout << "f[" << i << "] = ";
        //std::cout << "((tau * n_dot[" << i << "]) + (D * n[" << i << "])) / K  = ";
        //std::cout << "a[" << i << "] = ";

        for (int j=0; j<7; j++) {
			//std::cout << "\tj = " << j << "\n";
            if (i == 0) {
                v[i][j] = 0;
                a[i][j] = (x[i+1][j] - x[i][j] - v[i][j] * (t[i+1] - t[i])) 
                            * 2.0 / (t[i+1]*t[i+1] - t[i]*t[i]);
            } else if (i == t.size() - 1) {
                a[i][j] = 0;
                v[i][j] = v[i-1][j] + a[i-1][j] * (t[i] - t[i-1]);
            } else {
                v[i][j] = v[i-i][j] + a[i-1][j] * (t[i] - t[i-1]);
                a[i][j] = (x[i+1][j] - x[i][j] - v[i][j] * (t[i+1] - t[i])) 
                            * 2.0 / (t[i+1]*t[i+1] - t[i]*t[i]);
            }    
            n[i][j] = tau * v[i][j];
            n_dot[i][j] = tau * a[i][j];
            f[i][j] = ((tau * n_dot[i][j]) + (D * n[i][j])) / K
                        + (g[j] - x_0[j]) * s[i]
                        - (g[j] - x[i][j]);
            //std::cout << f[i][j] << " ";
            //std::cout << ((tau * n_dot[i][j]) + (D * n[i][j])) / K << " ";
            //std::cout << a[i][j] << " ";
        }
        //std::cout << "\n";
        //if (i < f.size() - 1) {
            //std::cout << "v[" << i << "] = ";
            //for (int j=0; j<7; j++) {
                //std::cout << v[i][j] << " ";
            //}
            //std::cout << "\nd[" << i << "] = ";
            //for (int j=0; j<7; j++) {
                //std::cout << x[i+1][j] - x[i][j] << " ";
            //}
            //std::cout << "\nv[" << i << "] * dt = ";
            //for (int j=0; j<7; j++) {
                //std::cout << v[i][j] * (t[i+1] - t[i]) << " ";
            //}
            //std::cout << "\ndt^2 = " << (t[i+1] - t[i]) * (t[i+1] - t[i]);
            //std::cout << "\na[" << i << "] = ";
            //for (int j=0; j<7; j++) {
                //std::cout << a[i][j] << " ";
            //}
            //std::cout << "\n\n";   
        //}
    }
    std::cout << "Learnt DMP...\n";
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
    while (translation_diff(velocities, get_zero_vector(7)) > VEL_THRESH) {
        std::cout << "Dangerous!!!";
        for (int i=0; i<7; i++) {
			velocities[i] /= 2;
		}
    }
    return velocities;
}

std::vector<double> normalize_quaternion(std::vector<double> quaternion) {
	double magnitude = quaternion[3] + quaternion[4] + quaternion[5] + quaternion[6];
    std::vector<double> normalized_quaternion = std::vector<double>(quaternion);
    for (int i=3; i<7; i++) {
        //normalized_quaternion[i] /= magnitude ;
        normalized_quaternion[i] = 0;
    }
    return normalized_quaternion;
}

void calculate_trajectory() {
	std::vector< std::vector<double> > empty_vector1;
	traj_x = empty_vector1;
	std::vector< std::vector<double> > empty_vector2;
	traj_v = empty_vector2;
	std::vector< std::vector<double> > empty_vector3;
	traj_a = empty_vector3;
	std::vector< std::vector<double> > empty_vector4;
	traj_f = empty_vector4;
	std::vector< std::vector<double> > empty_vector5;
	traj_n = empty_vector5;
	
	std::vector<double> cur_x = x_0, prev_x, cur_v, cur_f, cur_a, 
        next_v, cur_n, cur_n_dot, next_x, next_n;
	double cur_t, cur_s;
	bool first_point = true;
	
    //double delta_t = 1000.0 / ROS_RATE;
    double delta_t = 1.0 / ROS_RATE;
    //double delta_t = 20;
    std::cout << "delta_t = " << delta_t << "\n";
    std::cout << "K = " << K << "\n";
    
	int cc =0;
	while (translation_diff(cur_x, g) > POSE_DIFF_THRESH) {
		//std::cout << "Distance to go: " << translation_diff(cur_x, g) << "\n";
		cc++;
		if (cc == 5000){
			break;
		}
		
	    if (first_point) {
            cur_t = 0;
			first_point = false;
			prev_x = cur_x = x_0;
			cur_v = get_zero_vector(7);
            cur_n = get_zero_vector(7);
			cur_t = 0;
		}
	    cur_s = calc_s(cur_t);
        cur_f = calc_f(cur_s);
        
        //std::cout << "t = " << cur_t << ", s = " << cur_s << ", f = ";
        //for (int i=0; i<7; i++) {
            //std::cout << cur_f[i] << " ";
        //}
        //std::cout << "\n";
        //std::cin.get();
        
        cur_n_dot = get_zero_vector(7);
        cur_a = get_zero_vector(7);
        next_v = get_zero_vector(7);
        next_n = get_zero_vector(7);
        next_x = get_zero_vector(7);
        
        for (int i=0; i<7; i++) {
            cur_n_dot[i] = (K * (g[i] - cur_x[i]) - D * cur_n[i] - 
                K * (g[i] - x_0[i]) * cur_s + K * cur_f[i]) / tau;
            cur_a[i] = cur_n_dot[i] / tau;
            next_v[i] = cur_v[i] + cur_a[i] * delta_t;
            next_n[i] = tau * next_v[i];
            //next_x[i] = cur_x[i] + cur_v[i] * delta_t + 0.5 * cur_a[i] * delta_t * delta_t;
            next_x[i] = cur_x[i] + ((next_v[i]*next_v[i] - cur_v[i]*cur_v[i]) / 2*cur_a[i]);
        }
       
        next_v = normalize_quaternion(next_v);
        //std::cout << "Before safety...\n";
        next_v = make_velocities_safe(next_v);

		//std::cout << "next_v: ";
		//for (int i=0; i<7; i++) {
			//std::cout << next_v[i] << " ";
		//}
		//std::cout << "\n";

		//std::cout << "cur_x: ";
		//for (int i=0; i<7; i++) {
			//std::cout << cur_x[i] << " ";
		//}
		//std::cout << "\n";

		// Adding a point to the rviz marker
		visualization_msgs::Marker marker = create_marker(cur_x);
		marker_array.markers.push_back(marker);
        
        traj_x.push_back(cur_x);
        traj_v.push_back(cur_v);
        traj_a.push_back(cur_a);
        traj_f.push_back(cur_f);
        traj_n.push_back(cur_n);
        
        prev_x = cur_x;
        cur_x = next_x;
        cur_v = next_v;
        cur_n = next_n;
        cur_t += delta_t;
    }
}

void remove_redundant_points() {
    std::vector<int> indices_to_remove;
    for (int i=0; i<t.size(); i++) {
        if (magnitude_diff(x[i], x[0]) < NEAR_ZERO) {
            indices_to_remove.push_back(i);
        }    
    }
    std::vector<int> end_indices_to_remove;
    end_indices_to_remove.push_back(t.size()-1);
    for (int i=t.size()-2; i>=0; i--) {
        if (magnitude_diff(x[i], x[t.size()-2]) < NEAR_ZERO) {
            end_indices_to_remove.push_back(i);
        }    
    }
    for (int i=end_indices_to_remove.size()-1; i>=0; i--) {
        indices_to_remove.push_back(end_indices_to_remove[i]);
    }
    
    std::cout << "Indices to remove : ";
    for (int j=0; j<indices_to_remove.size(); j++) {
        std::cout << indices_to_remove[j] << " " ;
    }
    std::cout << "\n";
    
    std::vector< std::vector<double> > new_x;
    std::vector<double> new_t;
    int j = 0;
    for (int i=0; i<t.size(); i++) {
        if (i < indices_to_remove[j]) {
            new_x.push_back(x[i]);
            new_t.push_back(t[i]);
        } else {
            // This is an element to remove. Now search for the next one.
            j++;
        }
    }
    double subtract_val = new_t[0];
    for (int i=0; i<new_t.size(); i++) {
        new_t[i] -= subtract_val;
    }
    x = new_x;
    t = new_t;
}

void replay_calculated_trajectory(int rateHertz) {
	ros::Rate r(rateHertz);
	
	//int freq_to_use = (1000/rateHertz) / TRAJ_TIME_STEP;
	
	for (int i=0; i<traj_v.size(); i++) {
		std::cout << "Going to publish velocities: ";
		for (int j=0; j<7; j++) {
			std::cout << traj_v[i][j] << " ";
		}
		std::cout << "\n";
        
        if (!near_zero_vel(traj_v[i])) {
			std::cout << "Converting to message...\n";
			geometry_msgs::TwistStamped velocity_msg = get_velocity_msg(traj_v[i]);
			std::cout << "Going to publish \n" << velocity_msg << "\n";
			pub_velocity.publish(velocity_msg);
			r.sleep();
			ros::spinOnce();
			std::cout << "Came out of sleep\n";
			
			std::cout << "\nExpected current pose: ";
			for (int k=0; k<3; k++) {
				std::cout << traj_x[i][k] << " ";
			}
			std::cout << "\nGoal: ";
			for (int k=0; k<3; k++) {
				std::cout << g[k] << " ";
			}
			std::cout << "\n\n";
			
			
			std::cout << "End of loop\n";
		} else {
			std::cout << "Not publishing zero velocity...\n";
		}
	}
	
	geometry_msgs::TwistStamped zero_velocity_msg = get_velocity_msg(get_zero_vector(7));
	pub_velocity.publish(zero_velocity_msg);
	r.sleep();
	ros::spinOnce();
	
}

int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "subscriber");
	
	ros::NodeHandle n;

    //ros::AsyncSpinner spinner(1);
    //spinner.start();	
	//create subscriber to tool position topic - Cartesian end-effector position
	//ros::Subscriber sub_tool = n.subscribe("/mico_arm_driver/out/tool_position", 1, toolpos_cb);
	
	// Subscribe to the code that will publish a keyboard interrupt when the demo is over
	//ros::Subscriber sub = n.subscribe("keyboard_interrupt", 1000, keyboard_interrupt_cb);

	//publish velocities
	pub_velocity = n.advertise<geometry_msgs::TwistStamped>("/mico_arm_driver/in/cartesian_velocity", 10);

	//std::cout << "Move robot to start position and press Enter to start recording";
	//std::cin.get();
	//std::cout << "Recording demo. Press enter in the keyboard interrupter to stop...\n";

	// Publisher to publish in RViz
	ros::Publisher marker_pub =  
		n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

    std::cout << "Enter K: ";
    std::cin >> K;

    // Read demonstrated trajectory from file
    std::ifstream my_file; 
    my_file.open("Trajectory.csv");
    double val;
    while (!my_file.eof()) {
        std::vector<double> pose;
        for (int i=0; i<7; i++) {
            my_file >> val;
            //std::cout << "Read " << val << " from Trajectory.csv\n";
            //std::cin.get();
            if (i >= 3) {
                val = 0;
            }
            pose.push_back(val);
        }
        x.push_back(pose);
    }

    int time_val;
    std::ifstream time_file;
    time_file.open("Time.txt");
    while(!time_file.eof()) {
        time_file >> time_val;
        //std::cout << "Read " << time_val << " from Time.txt\n";
        t.push_back(time_val);
    }
    
	convert_time_to_sec();

    std::cout << "Finished reading files...\n";
    std::cout << "x.size() = " << x.size() << "\n";
    std::cout << "t.size() = " << t.size() << "\n";

    remove_redundant_points();

    std::cout << "Removing redundant points.. \n";
    std::cout << "x.size() = " << x.size() << "\n";
    std::cout << "t.size() = " << t.size() << "\n";

	set_x_0_and_g();
	tau = t[t.size() - 1];
	std::cout << "tau = " << tau << "\n";

    learn_dmp();
    
    std::cout << "Calculated DMP values - \n";
    
    //for (int i=0; i<t.size(); i++) {
        //std::cout << "s[" << i << "] = " << s[i] << "\n";
        //std::cout << "x[" << i << "] = ";
        //for (int j=0; j<7; j++) {
            //std::cout << x[i][j] << " " ;
        //}
        //std::cout << "\n";
        //std::cout << "v[" << i << "] = ";
        //for (int j=0; j<7; j++) {
            //std::cout << v[i][j] << " " ;
        //}
        //std::cout << "\n";
        //std::cout << "a[" << i << "] = ";
        //for (int j=0; j<7; j++) {
            //std::cout << a[i][j] << " " ;
        //}
        //std::cout << "\n";
        //std::cout << "f[" << i << "] = ";
        //for (int j=0; j<7; j++) {
            //std::cout << f[i][j] << " " ;
        //}
        //std::cout << "\n";
        //std::cout << "d = " << magnitude_diff(x[i], g) << "\n";
        //std::cout << "\n\n";
    //}
    
    calculate_trajectory();

    std::cout << "Calculated trajectory - \n";
    std::cout << "traj_x.size() = " << traj_x.size() << "\n";
    std::cout << "Distance of last point to goal: " << translation_diff(traj_x[traj_x.size() - 1], g) << "\n";
    
    //for (int i=0; i<traj_x.size(); i++) {
        //std::cout << "s[" << i << "] = " << s[i] << "\n";
        //std::cout << "traj_x[" << i << "] = ";
        //for (int j=0; j<7; j++) {
            //std::cout << traj_x[i][j] << " " ;
        //}
        //std::cout << "\n";
        //std::cout << "traj_v[" << i << "] = ";
        //for (int j=0; j<7; j++) {
            //std::cout << traj_v[i][j] << " " ;
        //}
        //std::cout << "\n";
        //std::cout << "traj_a[" << i << "] = ";
        //for (int j=0; j<7; j++) {
            //std::cout << traj_a[i][j] << " " ;
        //}
        //std::cout << "\n";
        //std::cout << "traj_f[" << i << "] = ";
        //for (int j=0; j<7; j++) {
            //std::cout << traj_f[i][j] << " " ;
        //}
        //std::cout << "\n";
        //std::cout << "d = " << magnitude_diff(x[i], g) << "\n";
        //std::cout << "\n\n";
    //}
    
    
    
	//recording = true;
	//int listenRateHertz = 50;
	//ros::Rate r(listenRateHertz);
    //gettimeofday(&start_of_demo, NULL); 
	//while (ros::ok() && !stopRecordingDemo) {
		//ros::spinOnce();
	//}
    //tau = get_last_pose_time_ms() / 1000;
	//recording = false;

	//std::cout << "Stopped recording demo.\n";
	//std::cout << "Recorded " << x.size() << " frames\n";
    
    //if (x.size() != t.size()) {
        //std::cout << "x.size() = " << x.size() << ", t.size() = " << t.size() << "\n";
        //exit(1);
    //} 
    
    //set_x_0_and_g();
	//learn_dmp();
    
    //calculate_trajectory();
    
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
    
    std::cout << "Exiting program...\n";    
    //// Reset tau for desired duration of demo
    
	std::cout << "Move robot to start position and press enter to replay demo : ";
	std::cin.get();
	
	//replay_calculated_trajectory(ROS_RATE);
	
	std::cout << "Replayed demo...";
}
