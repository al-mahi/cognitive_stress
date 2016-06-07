/*
 * dispEquationTest.cpp
 *
 *  Created on: Mar 30, 2015
 *      Author: matt
 */

#include "normEquTest.h"

int main(int argc, char** argv) {

	usePoints = (argc > 1 && std::string(argv[0]).compare("p"));

	if (usePoints) {
		std::cout << "Using points file!\n";
		readPoints();
		std::cout << "Done.\n";
	}

	std::string readLine;
	std::stringstream ss;
	std::vector<double> toAdd;
	float x, y, z;

	if (!usePoints) {
		while (true) {

			std::cout << "Enter p1 as {x, y, z}\n";
			std::cin >> x >> y >> z;
			printf("Read: {%.6f, %.6f, %.6f}\n\n", x, y, z);
			toAdd.push_back(x);
			toAdd.push_back(y);
			toAdd.push_back(z);
			points.push_back(toAdd);
			toAdd.clear();

			std::cout << "Enter p2 as {x, y, z}\n";
			std::cin >> x >> y >> z;
			printf("Read: {%.6f, %.6f, %.6f}\n\n", x, y, z);
			toAdd.push_back(x);
			toAdd.push_back(y);
			toAdd.push_back(z);
			points.push_back(toAdd);
			toAdd.clear();

			std::cout << "Enter p3 as {x, y, z}\n";
			std::cin >> x >> y >> z;
			printf("Read: {%.6f, %.6f, %.6f}\n\n", x, y, z);
			toAdd.push_back(x);
			toAdd.push_back(y);
			toAdd.push_back(z);
			points.push_back(toAdd);
			toAdd.clear();
			std::cout << '\n';
			computeDisparity();
			points.clear();
		}
	} else {

		int ip1, ip2;

		while(true) {
			std::cout << "Enter point indices for p1 and p2:\n";
			std::cin >> ip1 >> ip2;
			points.push_back(pointsFromFile[ip1]);
			points.push_back(pointsFromFile[ip2]);

			std::cout << "Enter third point as {x, y, z}:\n";
			std::cin >> x >> y >> z;
			toAdd.push_back(x);
			toAdd.push_back(y);
			toAdd.push_back(z);
			points.push_back(toAdd);
			toAdd.clear();
			std::cout << '\n';
			computeDisparity();
			points.clear();
		}
	}

}

void readPoints() {

	std::cout << "Reading points..\n";

	std::vector<double> toAdd;
	std::string readLine;
	float x, y, z;

	std::ifstream inFile;
	inFile.open("/home/matt/ros_testing/src/odom_measure/include/points.txt");

	if(!inFile.is_open()) {
		std::cerr << "ERROR! Could not open points file.\n";
		return;
	}

	while(std::getline(inFile, readLine)) {
		std::stringstream ss(readLine);

		ss >> x >> y >> z;
		toAdd.push_back(x);
		toAdd.push_back(y);
		toAdd.push_back(z);

		pointsFromFile.push_back(toAdd);
	}

	return;
}

void computeDisparity() {
	double x1, x2, x3, y1, y2, y3, z1, z2, z3;
	double numerator, denominator, distance;

	std::cout << "Computing Disparity\n";

	x1 = points[0][0];
	y1 = points[0][1];
	z1 = points[0][2];

	x2 = points[1][0];
	y2 = points[1][1];
	z2 = points[1][2];

	x3 = points[2][0];
	y3 = points[2][1];
	z3 = points[2][2];

	printf("\nPoints:\n p1 = {%.6f, %.6f, %.6f}\n p2 = {%.6f, %.6f, %.6f}\n p3 = {%.6f, %.6f, %.6f}\n\n", x1, y1, z1, x2, y2, z2, x3, y3, z3);

	// distance = |(y2 - y1)p3.x - (x2 - x1)p3.y + x2*y1 - x1*y2| / sqrt( (y2 - y1)^2 + (x2 - y1)^2)
	numerator = fabs( (y2 - y1) * x3 - (x2 - x1) * y3 + x2*y1 - y2*x1 );
	denominator = sqrt( pow(y2 - y1, 2.0) + pow(x2 - y2, 2.0) );
	distance = numerator / denominator;

	printf("Computed normal distance is: %.6f\n --> %.6f/%.6f\n", distance, numerator, denominator);
}
