#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
// Namespace std
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}


VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
	const vector<VectorXd> &ground_truth) {
	/**
	Done.
	* Calculate the RMSE here.
	*/

	VectorXd rsme(4);
	// Initialize rsme vector
	rsme << 0, 0, 0, 0;
	// Validity check
	if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
		cout << "Error, check estimations/ground truth sizes!" << endl;
		return rsme;
	}
	// Accumulate squared results
	for (unsigned int i = 0; i < estimations.size(); ++i) {
		VectorXd  residual = estimations[i] - ground_truth[i];
		// Coeff.-wise multiplication
		residual = residual.array() * residual.array();
		rsme += residual;
	}
	// Get the mean value
	rsme = rsme / estimations.size();
	// Squared root
	rsme = sqrt(rsme.array());
	return rsme;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	/**
	Done.
	* Calculate a Jacobian here.
	*/

	MatrixXd Hj(3, 4);
	float px = x_state[0];
	float py = x_state[1];
	float vx = x_state[2];
	float vy = x_state[3];
	// Pre-compute terms
	float helper1 = px*px + py*py;

	// Check if helper1 is close to zero or zero
	if (fabs(helper1) < 0.0001) {
		helper1 = 0.0001;
	}

	float helper2 = sqrt(helper1);
	float helper3 = (helper1*helper2);
	// Check division by zero
	// Get absolute value and compare to very small value
	if (fabs(helper1) < 0.0001) {
		cout << "Error - Division by zero detected!" << endl;
		return Hj;
	}
	// Compute the Jacobian matrix
	Hj << (px / helper2), (py / helper2), 0, 0,
		-(py / helper1), (px / helper1), 0, 0,
		py*(vx*py - vy*px) / helper3, px*(vy*px - vx*py) / helper3, px / helper2, py / helper2;
	
	return Hj;
}