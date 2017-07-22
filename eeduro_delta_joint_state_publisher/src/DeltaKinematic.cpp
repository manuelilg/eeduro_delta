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

std::shared_ptr<ForwardKinematicResult> DeltaKinematic::calculateForwardKinematic(const std::vector<double>& motorPositions) {
//	std::cout << __PRETTY_FUNCTION__ << " called" << std::endl;
	std::shared_ptr<ForwardKinematicResult> res = std::shared_ptr<ForwardKinematicResult>(new ForwardKinematicResult);
	std::vector<double> alphas;
	std::vector<Vector> link1s;
	std::vector<Position> endPointsLink1;
	for(int i = 0; i<3; i++) {
		alphas.push_back(getAlpha(motorPositions.at(i)));
//		std::cout << "Size alphas: " << alphas.size() << std::endl;
		link1s.push_back(getLink1(i, alphas.at(i)));
		endPointsLink1.push_back(getEndpointLink1(i, link1s.at(i)));
	}
//	std::cout << "Size endPointsLink1: " << endPointsLink1.size() << std::endl;

	Position tcp = getTCP(endPointsLink1);
	std::cout << "TCP: " << std::endl << tcp << std::endl;

	for(int j = 0; j < 3; j++) {
		Vector orthogonal = getOrthogonal(j, endPointsLink1.at(j), alphas.at(j));
		Vector link3 = tcp - endPointsLink1.at(j);
		Vector link3projection = getProjectVectorOntoPlane(link3, orthogonal);
		double beta = getBeta(link1s.at(j), link3projection, orthogonal);
		double gamma = getGamma(link3projection, link3, orthogonal);

		std::cout << "Arm: " << j << std::endl
				<< "ortho: " << orthogonal << std::endl
				<< "link3: " << link3 << std::endl
				<< "proj: " << link3projection << std::endl
				<< "beta: " << beta << std::endl
				<< "gamma: " << gamma << std::endl;
	}

	res->arm1.alpha = alphas.at(0);
	res->arm2.alpha = alphas.at(1);
	res->arm3.alpha = alphas.at(2);
	return res;
}

double DeltaKinematic::getAlpha(const double motorAngle) {
//	std::cout << __PRETTY_FUNCTION__ << " called" << std::endl;
	double offset = 0.0;
	double transmissionRate = 5103.0/387283.0;

	double alpha = motorAngle * transmissionRate + offset;

	return alpha;
}

Vector DeltaKinematic::getLink1(const double armNr, const double alpha) {
//	std::cout << __PRETTY_FUNCTION__ << " called" << std::endl;
	Vector link(0, -cos(alpha)*length_link1, sin(alpha)*length_link1);
	RotMatrix rotZ(armNr * 2*M_PI/3, Vector::UnitZ());

	return rotZ*link;
}

Position DeltaKinematic::getEndpointLink1(const double armNr, const Vector& link1) {
//	std::cout << __PRETTY_FUNCTION__ << " called" << std::endl;
	Vector armBase(0, -length_center2armBase , 0);
	RotMatrix rotZ(armNr * 2*M_PI/3, Vector::UnitZ());

	return rotZ*armBase + link1;
}

Position DeltaKinematic::getTCP(const std::vector<Position>& endPointsLink1) {
//	std::cout << __PRETTY_FUNCTION__ << " called" << std::endl;
	Circle circle1 = getIntersectionTwoSpheres(endPointsLink1.at(0), endPointsLink1.at(1), length_link3);
	Circle circle2 = getIntersectionPlaneSphere(circle1.center, circle1.normal, endPointsLink1.at(2), length_link3);
	Position toolCenterPoint = getIntersectionTwoCircles(circle1, circle2);

	return toolCenterPoint;
}

Circle DeltaKinematic::getIntersectionTwoSpheres(const Position& centerS1, const Position& centerS2, double radiusS) {
//	std::cout << __PRETTY_FUNCTION__ << " called" << std::endl;
	Circle circle;
	circle.center = centerS1 + (centerS2 - centerS1)/2;
	circle.normal = (centerS2 - centerS1).normalized();
	circle.radius = sqrt(pow(radiusS, 2) - pow((centerS2-centerS1).norm()/2, 2));
	return circle;
}

Circle DeltaKinematic::getIntersectionPlaneSphere(const Position& pointPlane,
		const Vector& normPlane,
		const Position& centerS,
		const double radiusS) {
//	std::cout << __PRETTY_FUNCTION__ << " called" << std::endl;
	Circle circle;

	Vector center2Plain = pointPlane - centerS;
	double distancePlane2Center = center2Plain.dot(normPlane);
	circle.center = centerS + distancePlane2Center*normPlane;

	circle.radius = sqrt(pow(radiusS, 2) - pow(distancePlane2Center, 2));
	circle.normal = normPlane;

	return circle;
}

Position DeltaKinematic::getIntersectionTwoCircles(const Circle& circle1, const Circle& circle2) {
//	std::cout << __PRETTY_FUNCTION__ << " called" << std::endl;
	Vector center1ToCenter2 = circle2.center - circle1.center;
	double distanceCenter1ToRadicalAxis = (pow(circle1.radius, 2) - pow(circle2.radius, 2) + pow(center1ToCenter2.norm(), 2)) / (2*center1ToCenter2.norm());

	Vector normalToCenterConnection = center1ToCenter2.cross(circle1.normal);
	double distanceCenterConnection2Intersection = sqrt(pow(circle1.radius, 2) - pow(distanceCenter1ToRadicalAxis, 2));

	Position intersection = circle1.center + center1ToCenter2.normalized()*distanceCenter1ToRadicalAxis + normalToCenterConnection.normalized()*distanceCenterConnection2Intersection;
	return intersection;
}

Vector DeltaKinematic::getOrthogonal(const double armNr, const Position& endPointLink1, const double alpha) {
//	std::cout << __PRETTY_FUNCTION__ << " called" << std::endl;
	RotMatrix rotZ = RotMatrix(armNr * 2*M_PI/3, Vector::UnitZ());
	RotMatrix rotArm = RotMatrix(alpha, Vector::UnitX());
	Position armBase = rotZ * Vector(0.0, -length_center2armBase, 0.0);
	Vector link1 = endPointLink1 - armBase;
	Vector orthogonal = link1.cross(rotZ*rotArm*Vector::UnitZ());
	orthogonal.normalize();

	return orthogonal;
}

Vector DeltaKinematic::getProjectVectorOntoPlane(const Vector& vector, const Vector& normalPlane) {
//	std::cout << __PRETTY_FUNCTION__ << " called" << std::endl;
	Vector rejection = normalPlane * vector.dot(normalPlane);
	return vector - rejection;
}

double DeltaKinematic::getBeta(const Vector& link1, const Vector& projectionLink3, const Vector& normal) {
//	std::cout << __PRETTY_FUNCTION__ << " called" << std::endl;
	double betaSign = (normal.dot(link1.cross(projectionLink3)) > 0.0 ? 1.0 : -1.0);
	double beta = betaSign * acos(link1.dot(projectionLink3) / (link1.norm() * projectionLink3.norm()) );
	return beta;
}

double DeltaKinematic::getGamma(const Vector& projectionLink3, const Vector& link3, const Vector& normal) {
//	std::cout << __PRETTY_FUNCTION__ << " called" << std::endl;
	double gammaSign = (normal.cross(projectionLink3).dot(projectionLink3.cross(link3))) > 0.0 ? 1.0 : -1.0;
	double gamma = gammaSign * acos(link3.dot(projectionLink3) / (link3.norm() * projectionLink3.norm()));
	return gamma;
}

}  // namespace delta_kinematic

