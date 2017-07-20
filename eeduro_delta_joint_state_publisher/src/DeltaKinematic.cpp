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

std::shared_ptr<ForwardKinematicResult> DeltaKinematic::calculateForwardKinematic(const double motor1Angle, const double motor2Angle, const double motor3Angle) {
	double alpha1 = getAlpha(motor1Angle);
	double alpha2 = getAlpha(motor2Angle);
	double alpha3 = getAlpha(motor3Angle);

	Eigen::Vector3d endPointArm1 = getEndpointLink1(1, alpha1);
	Eigen::Vector3d endPointArm2 = getEndpointLink1(2, alpha2);
	Eigen::Vector3d endPointArm3 = getEndpointLink1(3, alpha3);

	calculateTCP(endPointArm1, endPointArm2, endPointArm3);

	std::shared_ptr<ForwardKinematicResult> res = std::shared_ptr<ForwardKinematicResult>(new ForwardKinematicResult);
	res->arm1.alpha = alpha1;
	res->arm2.alpha = alpha2;
	res->arm3.alpha = alpha3;

	return res;
}

double DeltaKinematic::getAlpha(const double motorAngle) {
	double offset = 0.0;
	double transmissionRate = 5103.0/387283.0;

	double alpha = motorAngle * transmissionRate + offset;

	return alpha;
}

Eigen::Vector3d DeltaKinematic::getEndpointLink1(const int armNr, const double alpha) {
	Vector armBase(0, -length_center2armBase , 0);
	Vector link1(0, -cos(alpha)*length_link1, sin(alpha)*length_link1);
	RotMatrix rotZ((armNr-1) * 2*M_PI/3, Vector::UnitZ());

	return rotZ * (armBase + link1);
}

Position DeltaKinematic::calculateTCP(const Position& endPointLink1, const Position& endPointLink2, const Position& endPointLink3) {

	Circle circle1 = intersectionTwoSpheres(endPointLink1, endPointLink2, length_link3);
	Circle circle2 = intersectionPlaneSphere(circle1.center, circle1.normal, endPointLink3, length_link3);
	Position toolCenterPoint = intersectionTwoCircles(circle1, circle2);

	std::cout << toolCenterPoint << std::endl;

	return toolCenterPoint;
}

Circle DeltaKinematic::intersectionTwoSpheres(const Position& centerS1, const Position& centerS2, double radiusS) {
	Circle circle;
	circle.center = centerS1 + (centerS2 - centerS1)/2;
	circle.normal = (centerS2 - centerS1).normalized();
	circle.radius = sqrt(pow(radiusS, 2) - pow((centerS2-centerS1).norm()/2, 2));
	return circle;
}

Circle DeltaKinematic::intersectionPlaneSphere(const Position& pointPlane, const Vector& normPlane, const Position& centerS, const double radiusS) {
	Circle circle;

	Vector center2Plain = pointPlane - centerS;
	double distancePlane2Center = center2Plain.dot(normPlane);
	circle.center = centerS + distancePlane2Center*normPlane;

	circle.radius = sqrt(pow(radiusS, 2) - pow(distancePlane2Center, 2));
	circle.normal = normPlane;

	return circle;
}

Position DeltaKinematic::intersectionTwoCircles(const Circle& circle1, const Circle& circle2) {
	Vector center1ToCenter2 = circle1.center - circle2.center;
	double distanceCenter1ToRadicalAxis = (pow(circle1.radius, 2) - pow(circle2.radius, 2) + pow(center1ToCenter2.norm(), 2)) / (2*center1ToCenter2.norm());

	Vector normalToCenterConnection = center1ToCenter2.cross(circle1.normal);
	double distanceCenterConnection2Intersection = sqrt(pow(circle1.radius, 2) - pow(distanceCenter1ToRadicalAxis, 2));

	Position intersection = circle1.center + center1ToCenter2.normalized()*distanceCenter1ToRadicalAxis + normalToCenterConnection.normalized()*distanceCenterConnection2Intersection;

	return intersection;
}

}  // namespace delta_kinematic

