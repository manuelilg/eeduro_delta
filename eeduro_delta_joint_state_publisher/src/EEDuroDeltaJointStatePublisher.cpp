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
//	subscriber_ = nodeHandle_.subscribe("motor_angles", 1, &EEDuroDeltaJointStatePublisher::processMessage, this);
	subscriber_ = nodeHandle_.subscribe("/eeduro_delta/joint_states", 1, &EEDuroDeltaJointStatePublisher::processMessage, this);
}

EEDuroDeltaJointStatePublisher::~EEDuroDeltaJointStatePublisher() {

}

void EEDuroDeltaJointStatePublisher::processMessage(const sensor_msgs::JointState::ConstPtr& msg) {
	std::vector<double> motorPositions;
	for(int i = 0; i < 3; i++) {
		motorPositions.push_back(msg->position.at(i));
	}
	jointState_.header.stamp = msg->header.stamp;

	publishJointStates(motorPositions);
}

void EEDuroDeltaJointStatePublisher::publishJointStates(const std::vector<double>& motorPositions) {
	std::shared_ptr<delta_kinematic::ForwardKinematicResult> forwardKinResult;
	forwardKinResult = deltaKinematic_.calculateForwardKinematic(motorPositions);

	mapFKResults2EEDuroDelta(forwardKinResult, motorPositions);

	publisher_.publish(jointState_);
}

void EEDuroDeltaJointStatePublisher::mapFKResults2EEDuroDelta(const std::shared_ptr<delta_kinematic::ForwardKinematicResult> fKinResult, const std::vector<double>& motorPositions) {
	jointState_.name.resize(3*4+1);
	jointState_.position.resize(3*4+1);

	int pos = 0;
	for(int i = 0; i < 3; i++) {
		std::string armName = "arm" + std::to_string(i+1);
		jointState_.name[pos] = armName + "_motor_joint";
		jointState_.position[pos++] = motorPositions.at(i);

		jointState_.name[pos] = armName + "_joint0";
		jointState_.position[pos++] = fKinResult->arms.at(i).alpha - alphaOffset;

		jointState_.name[pos] = armName + "_joint1";
		jointState_.position[pos++] = fKinResult->arms.at(i).beta - betaOffset;

		jointState_.name[pos] = armName + "_joint2_1";
		jointState_.position[pos++] = fKinResult->arms.at(i).gamma - gammaOffset;

		if(i == 0) {
			jointState_.name[pos] = armName + "_joint4";
			jointState_.position[pos++] = fKinResult->arms.at(i).delta - deltaOffset;
		}
	}

}

void EEDuroDeltaJointStatePublisher::publishTestMessage() {
	jointState_.header.stamp = ros::Time::now();
	jointState_.name.resize(19);
	jointState_.position.resize(19);

	int pos = 0;
		for(int i = 0; i < 3; i++) {
		std::string armName = "arm" + std::to_string(i+1);
		jointState_.name[pos] = armName + "_motor_joint";
		jointState_.position[pos++] = 0;

		jointState_.name[pos] = armName + "_joint0";
		jointState_.position[pos++] = 0;

		jointState_.name[pos] = armName + "_joint1";
		jointState_.position[pos++] = 0;

		jointState_.name[pos] = armName + "_joint2_1";
		jointState_.position[pos++] = 0;

		jointState_.name[pos] = armName + "_joint2_2";
		jointState_.position[pos++] = 0;

		jointState_.name[pos] = armName + "_joint3_1";
		jointState_.position[pos++] = 0;

		if(i == 0) {
			jointState_.name[pos] = armName + "_joint4";
			jointState_.position[pos++] = 0;
		}
	}

	ROS_INFO("publish now jointState");

	publisher_.publish(jointState_);
}

} /* namespace */



