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
	subscriber_ = nodeHandle_.subscribe("motor_angles", 1, &EEDuroDeltaJointStatePublisher::processMessage, this);

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

void EEDuroDeltaJointStatePublisher::processMessage(const sensor_msgs::JointState::ConstPtr& msg) {
	std::cout << __PRETTY_FUNCTION__ << " called" << std::endl;

	std::vector<double> motorPositions;
	for(int i = 0; i < 3; i++) {
		motorPositions.push_back(msg->position.at(i));
	}

	std::shared_ptr<delta_kinematic::ForwardKinematicResult> forwardKinResult;
	forwardKinResult = deltaKinematic_.calculateForwardKinematic(motorPositions);
	jointState_.header.stamp = msg->header.stamp;
	jointState_.name.resize(6);
	jointState_.position.resize(6);

	jointState_.name[0] = "arm1_motor_joint";
	jointState_.name[1] = "arm2_motor_joint";
	jointState_.name[2] = "arm3_motor_joint";
	jointState_.position[0] = motorPositions.at(0);
	jointState_.position[1] = motorPositions.at(1);
	jointState_.position[2] = motorPositions.at(2);

	jointState_.name[3] = "arm1_joint0";
	jointState_.name[4] = "arm2_joint0";
	jointState_.name[5] = "arm3_joint0";
	jointState_.position[3] = forwardKinResult->arm1.alpha;
	jointState_.position[4] = forwardKinResult->arm2.alpha;
	jointState_.position[5] = forwardKinResult->arm3.alpha;

	publisher_.publish(jointState_);
}

} /* namespace */



