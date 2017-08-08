/*
 * EEDuroDeltaJointStatePublisher.hpp
 *
 *  Created on: Jul 18, 2017
 *      Author: manuel
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "DeltaKinematic.hpp"

namespace eeduro_delta_joint_state_publisher {

class EEDuroDeltaJointStatePublisher {
public:
	EEDuroDeltaJointStatePublisher(ros::NodeHandle& nodeHandle);
	virtual ~EEDuroDeltaJointStatePublisher();

	void publishTestMessage();

private:
	void processMessage(const sensor_msgs::JointState::ConstPtr& msg);
	void publishJointStates(const std::vector<double>& motorPositions);
	void mapFKResults2EEDuroDelta(const std::shared_ptr<delta_kinematic::ForwardKinematicResult> fKinResult, const std::vector<double>& motorPositions);

private:
	ros::NodeHandle nodeHandle_;
	ros::Subscriber subscriber_;
	ros::Publisher publisher_;
	sensor_msgs::JointState jointState_;
	delta_kinematic::DeltaKinematic deltaKinematic_;

	double alphaOffset = 0.0;
	double betaOffset =  -2.49368823859929645;
	double gammaOffset = 0.0;
	double deltaOffset = 5.63528089218908956;
};


} /* namespace */


