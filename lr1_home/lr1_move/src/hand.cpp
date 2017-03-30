#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "serial/serial.h"
//#include<iostream>
#include <string>
#include <math.h>

int JointNum = 0;
double HandJoint1ValueP = 0, HandJoint2ValueP = 0;
serial::Serial my_serial;

void handCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	int i, hand_joint1_index = -1, hand_joint2_index = -1;
	unsigned char send_data[3];

	JointNum = msg->name.size();
	for(i = 0; i < JointNum; i++)
	{
		if(msg->name[i] == "arm_wrist_joint")
			hand_joint1_index = i;
		else if(msg->name[i] == "saw_joint")
			hand_joint2_index = i;
	}
	
	if(hand_joint1_index >= 0 
		&& fabs(msg->position[hand_joint1_index] - HandJoint1ValueP) > 0.001
	  )
	{
		HandJoint1ValueP = msg->position[hand_joint1_index];
//		ROS_INFO("HandJoint1: [%f]", HandJoint1ValueP);
		send_data[0] = 0xA1;
		send_data[1] = (unsigned char)(HandJoint1ValueP*180/M_PI/0.9 + 50);
		send_data[2] = ~(send_data[0] + send_data[1]);
		my_serial.write(send_data, 3);
	}
	
	if(hand_joint2_index >= 0 
		&& fabs(msg->position[hand_joint2_index] - HandJoint2ValueP) > 0.001
	  )
	{
		HandJoint2ValueP = msg->position[hand_joint2_index];	
//		ROS_INFO("HandJoint2: [%f]", HandJoint2ValueP);
		send_data[0] = 0xA2;
		send_data[1] = (unsigned char)(HandJoint2ValueP*180/M_PI/0.9 + 50);
		send_data[2] = ~(send_data[0] + send_data[1]);
		my_serial.write(send_data, 3);
	}
	
   
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hand_ctr");
	
	my_serial.setPort("/dev/ttyS0");
	my_serial.setBaudrate(9600);
	my_serial.setTimeout(serial::Timeout::max(), 1000, 0, 1000, 0);
	my_serial.open();
	if(my_serial.isOpen())
		ROS_INFO("COM1 Opened!");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/joint_states", 1, handCallback);

	ros::spin();

	return 0;
}
