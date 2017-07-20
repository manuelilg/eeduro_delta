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

private:
	ros::NodeHandle nodeHandle_;
	ros::Subscriber subscriber_;
	ros::Publisher publisher_;
	sensor_msgs::JointState jointState_;
	delta_kinematic::DeltaKinematic deltaKinematic_;

};


} /* namespace */


