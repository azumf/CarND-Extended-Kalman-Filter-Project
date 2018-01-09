#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  Done.
    * predict the state
  */
	// Motion vector u = 0
	x_ = F_ * x_;
	MatrixXd Ft_ = F_.transpose();
	P_ = F_ * P_ * Ft_ + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  Done.
    * update the state by using Kalman Filter equations
  */
	// Update step for Laser measurement
	// Check if px or py are close to or zero
	float px = x_[0];
	float py = x_[1];

	if (fabs(px) < 0.0001 && fabs(py) < 0.0001) {
		px = 0.0001;
		py = 0.0001;
	}
	VectorXd z_pred = H_ * x_;
	VectorXd y_ = z - z_pred;
	// Normalize
	y_[1] = atan2(sin(y_[1]), cos(y_[1]));
	MatrixXd Ht_ = H_.transpose();
	// 
	MatrixXd S_ = H_ * P_ * Ht_ + R_;
	MatrixXd Si_ = S_.inverse();
	// Kalman gain
	MatrixXd K_ = P_ * Ht_ * Si_;
	// New state
	x_ = x_ + (K_ * y_);
	// Identity matrix
	long x_size = x_.size();
	MatrixXd I_ = MatrixXd::Identity(x_size, x_size);
	P_ = (I_ - K_*H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

	// atan2() outputs values between -pi and pi
	VectorXd z_pred = VectorXd(3);
	float px = x_[0];
	float py = x_[1];
	float vx = x_[2];
	float vy = x_[3];

	// Check if px or py are close to or zero
	if (fabs(px) < 0.0001 && fabs(py) < 0.0001) {
		px = 0.0001;
		py = 0.0001;
	}

	z_pred[0] = sqrt(px*px + py*py);
	z_pred[1] = atan2(py,px);
	z_pred[2] = (px*vx + py*vy) / z_pred[0];

	// y = z - h'(x)
	VectorXd y_ = z - z_pred;
	// Normalize
	y_[1] = atan2(sin(y_[1]), cos(y_[1]));
	MatrixXd Ht_ = H_.transpose();
	MatrixXd S_ = H_ * P_ * Ht_ + R_;
	MatrixXd Si_ = S_.inverse();
	MatrixXd K_ = P_ * Ht_ * Si_;
	// Update
	x_ = x_ + (K_ * y_);
	long x_size = x_.size();
	MatrixXd I_ = MatrixXd::Identity(x_size, x_size);
	P_ = (I_ - K_*H_) * P_;
}
