/*
 * EEDuroDeltaJointStatePublisher.cpp
 *
 *  Created on: Jul 18, 2017
 *      Author: manuel
 */
#include "eeduro_delta_joint_state_publisher/EEDuroDeltaJointStatePublisher.hpp"

namespace eeduro_delta_joint_state_publisher {

EEDuroDeltaJointStatePublisher::EEDuroDeltaJointStatePublisher(ros::NodeHandle& nodeHandle) :
		nodeHandle_(nodeHandle) {
	publisher_ = nodeHandle_.advertise<sensor_msgs::JointState>("joint_states", 1);
	//subscriber_ = nodeHandle_.subscribe("")

}

EEDuroDeltaJointStatePublisher::~EEDuroDeltaJointStatePublisher() {

}

void EEDuroDeltaJointStatePublisher::publishTestMessage() {
	jointState_.header.stamp = ros::Time::now();
	jointState_.name.resize(1);
	jointState_.position.resize(1);
	jointState_.name[0] = "arm1_joint0";
	jointState_.position[0] = -0.5;

	ROS_INFO("publish now jointState");

	publisher_.publish(jointState_);
}

} /* namespace */



