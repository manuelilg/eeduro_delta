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

std::shared_ptr<ForwardKinematicResult> DeltaKinematic::calculateForwardKinematic(const std::vector<double>& motorPositions) {
	std::shared_ptr<ForwardKinematicResult> res = std::shared_ptr<ForwardKinematicResult>(new ForwardKinematicResult);
	std::vector<double> alphas;
	std::vector<Vector> link1s;
	std::vector<Position> endPointsLink1;
	std::vector<double> betas;
	std::vector<double> gammas;

	for(int i = 0; i < 3; i++) {
		alphas.push_back(motorPositions.at(i));
		link1s.push_back(getLink1(i, alphas.at(i)));
		endPointsLink1.push_back(getEndpointLink1(i, link1s.at(i)));
	}

	Position tcp = getTCP(endPointsLink1);

	std::cout << "TCP x: " << tcp[0] << "y: " << tcp[1] << "z: " << tcp[2] << std::endl;

	for(int j = 0; j < 3; j++) {
		Vector orthogonal = getOrthogonal(j, link1s.at(j), alphas.at(j));
		Vector link3 = tcp - endPointsLink1.at(j);
		Vector link3projection = getProjectVectorOntoPlane(link3, orthogonal);
		betas.push_back(getBeta(link1s.at(j), link3projection, orthogonal));
		gammas.push_back(getGamma(link3projection, link3, orthogonal));
	}

	for(int k = 0; k < 3; k++) {
		ArmAngles armAngles;
		armAngles.alpha = alphas.at(k);
		armAngles.beta = betas.at(k);
		armAngles.gamma = gammas.at(k);
		res->arms.push_back(armAngles);
	}
	res->arms.at(0).delta = M_PI - res->arms.at(0).alpha - res->arms.at(0).beta;

	return res;
}

Vector DeltaKinematic::getLink1(const double armNr, const double alpha) {
	Vector link(0, -cos(alpha)*length_link1, sin(alpha)*length_link1);
	RotMatrix rotZ(armNr * 2*M_PI/3, Vector::UnitZ());

	return rotZ*link;
}

Position DeltaKinematic::getEndpointLink1(const double armNr, const Vector& link1) {
	Vector armBase(0, -length_center2armBase , 0);
	RotMatrix rotZ(armNr * 2*M_PI/3, Vector::UnitZ());

	return rotZ*armBase + link1;
}

Position DeltaKinematic::getTCP(const std::vector<Position>& endPointsLink1) {
	Circle circle1 = getIntersectionTwoSpheres(endPointsLink1.at(0), endPointsLink1.at(1), length_link3);
	Circle circle2 = getIntersectionPlaneSphere(circle1.center, circle1.normal, endPointsLink1.at(2), length_link3);
	Position toolCenterPoint = getIntersectionTwoCircles(circle1, circle2);

	return toolCenterPoint;
}

Circle DeltaKinematic::getIntersectionTwoSpheres(const Position& centerS1, const Position& centerS2, double radiusS) {
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
	Circle circle;

	Vector center2Plain = pointPlane - centerS;
	double distancePlane2Center = center2Plain.dot(normPlane);
	circle.center = centerS + distancePlane2Center*normPlane;

	circle.radius = sqrt(pow(radiusS, 2) - pow(distancePlane2Center, 2));
	circle.normal = normPlane;

	return circle;
}

Position DeltaKinematic::getIntersectionTwoCircles(const Circle& circle1, const Circle& circle2) {
	Vector center1ToCenter2 = circle2.center - circle1.center;
	double distanceCenter1ToRadicalAxis = (pow(circle1.radius, 2) - pow(circle2.radius, 2) + pow(center1ToCenter2.norm(), 2)) / (2*center1ToCenter2.norm());

	Vector normalToCenterConnection = center1ToCenter2.cross(circle1.normal);
	double distanceCenterConnection2Intersection = sqrt(pow(circle1.radius, 2) - pow(distanceCenter1ToRadicalAxis, 2));

	Position intersection = circle1.center + center1ToCenter2.normalized()*distanceCenter1ToRadicalAxis + normalToCenterConnection.normalized()*distanceCenterConnection2Intersection;
	return intersection;
}

Vector DeltaKinematic::getOrthogonal(const double armNr, const Position& link1, const double alpha) {
	RotMatrix rotZ = RotMatrix(armNr * 2*M_PI/3, Vector::UnitZ());
	RotMatrix rotArm = RotMatrix(-alpha, Vector::UnitX());
	Vector orthogonal = link1.cross(rotZ*rotArm*Vector::UnitZ());
	orthogonal.normalize();

	return orthogonal;
}

Vector DeltaKinematic::getProjectVectorOntoPlane(const Vector& vector, const Vector& normalPlane) {
	Vector rejection = normalPlane * vector.dot(normalPlane);
	return vector - rejection;
}

double DeltaKinematic::getBeta(const Vector& link1, const Vector& projectionLink3, const Vector& normal) {
	double betaSign = (normal.dot(link1.cross(projectionLink3)) > 0.0 ? 1.0 : -1.0);
	double beta = betaSign * acos(link1.dot(projectionLink3) / (link1.norm() * projectionLink3.norm()) );
	return beta;
}

double DeltaKinematic::getGamma(const Vector& projectionLink3, const Vector& link3, const Vector& normal) {
	double gammaSign = (normal.cross(projectionLink3).dot(projectionLink3.cross(link3))) > 0.0 ? 1.0 : -1.0;
	double gamma = gammaSign * acos((float) (link3.dot(projectionLink3) / (link3.norm() * projectionLink3.norm())));

	return gamma;
}

std::vector<double> DeltaKinematic::inverse(const Position& tcp) {
	std::vector<double> alphas;

	Vector nullVec;
	for (int i = 0; i < 3; ++i) {
		Vector mountingPointLink1 = getEndpointLink1(i, nullVec); // method name is wrong for this case
		Vector mpL2TCP = tcp - mountingPointLink1;
		Vector projection = getProjectVectorOntoPlane(mpL2TCP, getOrthogonal(i, (Position) getLink1(i, 0.0), 0.0));
		double distPlane2Tcp = (mpL2TCP - projection).norm();
		double lengthProjectionLink3 = sqrt(length_link3*length_link3 - distPlane2Tcp*distPlane2Tcp);
		double alpha = M_PI/2 - (acos((lengthProjectionLink3*lengthProjectionLink3 - projection.norm()*projection.norm() - length_link1*length_link1) /(-2*projection.norm()*length_link1) ));
		alphas.push_back(alpha);
	}

	return alphas;
}

}  // namespace delta_kinematic

