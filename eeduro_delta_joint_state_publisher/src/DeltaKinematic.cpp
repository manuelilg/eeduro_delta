/*
 * DeltaKinematic.cpp
 *
 *  Created on: Jul 20, 2017
 *      Author: manuel
 */

#include "eeduro_delta_joint_state_publisher/DeltaKinematic.hpp"

namespace delta_kinematic {

DeltaKinematic::DeltaKinematic() {
}

DeltaKinematic::~DeltaKinematic() {
}

std::shared_ptr<ForwardKinematicResult> DeltaKinematic::calculateForwardKinematic(double motor1Angle, double motor2Angle, double motor3Angle) {
	std::shared_ptr<ForwardKinematicResult> res = std::shared_ptr<ForwardKinematicResult>(new ForwardKinematicResult);

	double alpha1 = getAlpha(motor1Angle);
	double alpha2 = getAlpha(motor2Angle);
	double alpha3 = getAlpha(motor3Angle);

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

}  // namespace delta_kinematic

