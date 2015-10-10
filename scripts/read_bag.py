#!/usr/bin/env python

import rospy, rosbag
import csv

def get_time_diff(last_secs, last_nsecs, secs, nsecs) :
    return (secs - last_secs) * 100000000 + (nsecs - last_nsecs)

def main() :
    # Load the bag
    bag = rosbag.Bag('Trajectory.bag')
    
    f = open('Trajectory.csv', 'w')
    writer = csv.writer(f, delimiter=' ')
    
    f2 = open('Time.txt', 'w')
    writer2 = csv.writer(f2, delimiter=',')
            
    first_reading = True
    
    for topic, msg, timestamp in bag.read_messages(topics=['/mico_arm_driver/out/tool_position']) :
        if first_reading :
            first_reading = False
            start_secs = timestamp.secs
            start_nsecs = timestamp.nsecs
        diff_secs = timestamp.secs - start_secs
        diff_nsecs = timestamp.nsecs - start_nsecs
        diff_ms = diff_secs * 1000 + diff_nsecs / 1000000
        writer2.writerow([diff_ms])
        #f2.write(str(diff_ms) + '\n')
        writer.writerow([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
		## Obtain positions of required joints from message	
		#grouped = zip(msg.name, msg.position, msg.velocity, msg.effort)
		#required_joint_values = [(name, position, velocity, effort) for (name, position, velocity, effort) in grouped if name in required_joint_names]
		#positions = [round(position, 6) for (name, position, velocity, effort) in required_joint_values]
		#position_differences = [abs(positions[i] - prev_positions[i]) for i in indices]
		#net_position_diff = sum(position_differences)

		## Check whether movement has started
		#if robot_moving == False and net_position_diff > 0.01 and loop_var > 1 :
			#robot_moving = True			
		
		## If robot is moving and more values need to be printed, check distance to last timestamp and print
		#if robot_moving == True and get_time_diff(last_secs, last_nsecs, timestamp.secs, timestamp.nsecs) > 100000000 : 
			#writer.writerow([timestamp] + positions)
			#last_secs = timestamp.secs
			#last_nsecs = timestamp.nsecs
			#num_printed += 1
			#if num_printed >= 10 :
				#break

		## Set prev_positions for next iteration
		#prev_positions = positions
		#loop_var += 1

if __name__ == '__main__':
	try :
		main()
	except rospy.ROSInterruptException :
		raise

