/*
 * DeltaKinematic.hpp
 *
 *  Created on: Jul 20, 2017
 *      Author: manuel
 */
#pragma once

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>

namespace delta_kinematic {

struct ToolCenterPoint {
	double x;
	double y;
	double z;
};

struct ArmAngles {
	double alpha;
	double beta;
	double gamma;
	double delta;
};

struct ForwardKinematicResult {

	ToolCenterPoint tcp;
	ArmAngles arm1;
	ArmAngles arm2;
	ArmAngles arm3;
};


class DeltaKinematic {
public:
	DeltaKinematic();
	virtual ~DeltaKinematic();

	std::shared_ptr<ForwardKinematicResult> calculateForwardKinematic(double motor1Angle, double motor2Angle, double motor3Angle);
private:
	double getAlpha(double motorAngle);
	Eigen::Vector3d getEndpointLink1(int armNr, double alpha);


private:
	double length_center2armBase = 0.1/sqrt(3) -0.028;
	double length_link1 = 0.05;
	double length_link3 = 0.1;

};

}  // namespace delta_forward_kinematic
