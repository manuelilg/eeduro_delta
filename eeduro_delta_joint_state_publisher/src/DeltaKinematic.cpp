/*
 * DeltaKinematic.cpp
 *
 *  Created on: Jul 20, 2017
 *      Author: manuel
 */

#include "eeduro_delta_joint_state_publisher/DeltaKinematic.hpp"

namespace delta_kinematic {

DeltaKinematic::DeltaKinematic() {
	std::cout << "include iostream active!!" << std::endl;
}

DeltaKinematic::~DeltaKinematic() {
}

std::shared_ptr<ForwardKinematicResult> DeltaKinematic::calculateForwardKinematic(double motor1Angle, double motor2Angle, double motor3Angle) {
	double alpha1 = getAlpha(motor1Angle);
	double alpha2 = getAlpha(motor2Angle);
	double alpha3 = getAlpha(motor3Angle);

	Eigen::Vector3d endPointArm1 = getEndpointLink1(1, alpha1);
	Eigen::Vector3d endPointArm2 = getEndpointLink1(2, alpha2);
	Eigen::Vector3d endPointArm3 = getEndpointLink1(3, alpha3);

	std::shared_ptr<ForwardKinematicResult> res = std::shared_ptr<ForwardKinematicResult>(new ForwardKinematicResult);
	res->arm1.alpha = alpha1;
	res->arm2.alpha = alpha2;
	res->arm3.alpha = alpha3;

	return res;
}

double DeltaKinematic::getAlpha(double motorAngle) {
	double offset = 0.0;
	double transmissionRate = 5103.0/387283.0;

	double alpha = motorAngle * transmissionRate + offset;

	return alpha;
}

Eigen::Vector3d DeltaKinematic::getEndpointLink1(int armNr, double alpha) {
	Eigen::Vector3d armBase(0, -length_center2armBase , 0);
	Eigen::Vector3d link1(0, -cos(alpha)*length_link1, sin(alpha)*length_link1);
	Eigen::AngleAxisd rotZ((armNr-1) * 2*M_PI/3, Eigen::Vector3d::UnitZ());

	return rotZ * (armBase + link1);
}

}  // namespace delta_kinematic

