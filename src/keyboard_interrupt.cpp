#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv) {
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("keyboard_interrupt", 1000);

	ros::Rate loop_rate(10);

	int count = 0;
    
    // Wait for key press
	std::cout << "Press any key to stop recording the demo \n";
	std::cin.get();

	while (ros::ok() && count <= 5) {
        // Publish a message a few times on the topic in case one of them
        // is lost
        
		std_msgs::String msg;
		std::stringstream ss;
		ss << "Stop";
		msg.data = ss.str();

		chatter_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}
