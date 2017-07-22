/*
 * eeduro_delta_joint_state_publisher_node.cpp
 *
 *  Created on: Jul 18, 2017
 *      Author: manuel
 */
#include <ros/ros.h>
#include "eeduro_delta_joint_state_publisher/EEDuroDeltaJointStatePublisher.hpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "eeduro_delta_joint_state_publisher");
	ros::NodeHandle nodeHandle("~"); //or ("~") for private NodeHandles
	ros::Rate loop_rate(1);

	eeduro_delta_joint_state_publisher::EEDuroDeltaJointStatePublisher ed(nodeHandle);

//	while(ros::ok()) {
//		ed.publishTestMessage();
//		loop_rate.sleep();
//	}

	ros::spin();

	return 0;
}




