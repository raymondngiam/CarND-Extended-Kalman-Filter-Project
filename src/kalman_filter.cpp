#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

// define constant for PI
const float PI = atan(1.0)*4;

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
  TODO:
    * predict the state
  */
	x_ = F_*x_;
	P_ = F_*P_*F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	VectorXd y = z - H_*x_;
	UpdateCore(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
	VectorXd z_(3);
	z_ << z(0), z(1), z(2);

	// Ensure phi falls between [-pi, pi]
	z_(1) = NormalizeAngle(z_(1));

	// Calculate h(x')
	float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
	float phi = atan2(x_(1), x_(0));  // returns value in range [-pi, pi]
	float rho_dot;
	if (fabs(rho) > 1e-6) {	// prevent divide by 0
		rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
	}
	else{
		rho_dot = 0.;
	}
	VectorXd hx(3);	
	hx << rho, phi, rho_dot;

	// Calculate y vector and normalize y(1), i.e. make -pi < y(1) < pi
	VectorXd y = z - hx;
	y(1) = NormalizeAngle(y(1));
	UpdateCore(y);
}

void KalmanFilter::UpdateCore(const VectorXd &y) {
  /**
  Measurement update function shared by KF and EKF
  */
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K =	P_ * Ht * Si;

	MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());

	//new state
	x_ = x_ + (K * y);
	P_ = (I - K * H_) * P_;
}

float KalmanFilter::NormalizeAngle(const float &phi_in) {
  /**
  Ensure phi angle in the range [-pi,pi]
  */
	float phi = phi_in;
	while (fabs(phi>PI)){
		if (phi > PI) {
			phi -= 2*PI;
		}
		else if (phi < -PI) {
			phi += 2*PI;
		}
	}
	return phi;
}
