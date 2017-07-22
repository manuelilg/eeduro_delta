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

#include "TypeDefs.hpp"

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

struct Circle {
	Position center;
	Vector normal;
	double radius;
};


class DeltaKinematic {
public:
	DeltaKinematic();
	virtual ~DeltaKinematic();

	std::shared_ptr<ForwardKinematicResult> calculateForwardKinematic(const std::vector<double>& motorPositions);
private:
	double getAlpha(const double motorAngle);
	Vector getLink1(const double armNr, const double alpha);
	Position getEndpointLink1(const double armNr, const Vector& link1);
	Position getTCP(const std::vector<Position>& endPointsLink1);
//	void calculateAngles(std::shared_ptr<ForwardKinematicResult> result, const std::vector<double>& alphas, const std::vector<Vector>& link1s, const std::vector<Position>& endPointsLink1, const Position& tcp);
	Circle getIntersectionTwoSpheres(const Position& centerS1, const Position& centerS2, const double radiusS);
	Circle getIntersectionPlaneSphere(const Position& pointPlane, const Vector& normPlane, const Position& centerS, const double radiusS);
	Position getIntersectionTwoCircles(const Circle& circle1, const Circle& circle2);
	Vector getOrthogonal(const double armNr, const Position& endPointLink1, const double alpha);
	Vector getProjectVectorOntoPlane(const Vector& vector, const Vector& normalPlane);
	double getBeta(const Vector& link1, const Vector& projectionLink3, const Vector& normal);
	double getGamma(const Vector& projectionLink3, const Vector& link3, const Vector& normal);

private:
	double length_center2armBase = 0.1/sqrt(3) -0.028;
	double length_link1 = 0.05;
	double length_link3 = 0.1;

};

}  // namespace delta_forward_kinematic
