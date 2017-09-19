/*
 * EEDuroDeltaJointStatePublisher.hpp
 *
 *  Created on: Jul 18, 2017
 *      Author: manuel
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

//#include "TypeDefs.hpp"

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

	void processCommand(const geometry_msgs::PoseStamped::ConstPtr& msg);

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

	ros::Subscriber commandSubscriber_;
	ros::Publisher arm1CommandPublisher_;
	ros::Publisher arm2CommandPublisher_;
	ros::Publisher arm3CommandPublisher_;
};


} /* namespace */


