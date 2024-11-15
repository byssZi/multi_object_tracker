#include "multi_object_tracker/ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
	
	// first measurement is used to initialize the UKF
	is_initialized_ = false;
		
	// if this is false, laser measurements will be ignored (except during init)
	use_laser_ = true;

	// if this is false, radar measurements will be ignored (except during init)
	use_radar_ = true;

	// initial state vector
	x_ = VectorXd(5);

	// initial covariance matrix
	P_ = MatrixXd(5, 5);

	// Process noise standard deviation longitudinal acceleration in m/s^2
	std_a_ = 1.0;
		
	// Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_ = M_PI/6.0;

	// Laser measurement noise standard deviation position1 in m
	std_laspx_ = 0.15;

	// Laser measurement noise standard deviation position2 in m
	std_laspy_ = 0.15;

	// Radar measurement noise standard deviation radius in m
	std_radr_ = 0.3;

	// Radar measurement noise standard deviation angle in rad
	std_radphi_ = 0.03;

	// Radar measurement noise standard deviation radius change in m/s
	std_radrd_ = 0.3;

	// dimension of state vector
	n_x_ = 5;
		
	// dimention of augmented state vector
	n_aug_ = 7;
		
	// spreading parameter
	lambda_ = 3 - n_aug_;
		
	// state vector
	x_ = VectorXd(n_x_);
	x_.fill(0.0);
		
	// state covariance matrix
	P_ = MatrixXd(n_x_, n_x_);
	P_.fill(0.0);
		
	// weights vector for sigma points
	weights_ = VectorXd(2*n_aug_+1);
	weights_(0) = lambda_/(lambda_+n_aug_);
	double w = 0.5/(lambda_+n_aug_);
	for (int i = 1; i < 2*n_aug_+1; ++i) {
		weights_(i) = w;
	}
		
	/**
	 TODO:

	Complete the initialization. See ukf.h for other member properties.

	Hint: one or more values initialized above might be wildly off...
	*/
}

UKF::~UKF() {}

/**
 * For first measurement, initializes x_ and P_.
 * For remaining measurements, calls prediction and update methods.
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

	// first measurement only used to initialize state vector, covariance matrix and timestamp
	if (!is_initialized_) {
		if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			
			double rho = meas_package.raw_measurements_[0];
			double phi = meas_package.raw_measurements_[1];
			double rho_dot = meas_package.raw_measurements_[2];

			// 计算目标物体在 x 和 y 方向上的速度分量
			double vx = rho_dot * cos(phi);
			double vy = rho_dot * sin(phi);

			// 计算合速度 v
			double v = sqrt(vx * vx + vy * vy);

			x_ <<	rho * cos(phi), // px
						rho * sin(phi), // py
						v, 							// v
						0,								// psi
						0;								// psi dot
			
			P_(0, 0) = 0.05; // best 0.05
			P_(1, 1) = 0.05; // best 0.05
			P_(2, 2) = 0.1; // best 0.1
			P_(3, 3) = 1.0;
			P_(4, 4) = 1.0;
			
			//cout << "radar measurement: ";
		} else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			
			x_ <<	meas_package.raw_measurements_[0], // px
						meas_package.raw_measurements_[1], // py
						0,																	 // v
						0,																	 // psi
						0;																	 // psi dot
			
			P_(0, 0) = 0.05; // best 0.05
			P_(1, 1) = 0.05; // best 0.05
			P_(2, 2) = 1.0;
			P_(3, 3) = 1.0;
			P_(4, 4) = 1.0;
			
			//cout << "lidar measurement: ";
		} else {
			cout << "Error: invalid meas_package.sensor_type_ during initialization" << endl;
			return;
		}
		
		previous_timestamp_ = meas_package.timestamp_;
		
		
		is_initialized_ = true;
		//cout << "px = " << x_(0) << ", py = " << x_(1) << endl;
		//cout << "UKF initialized" << endl;
		
		return;
	}
	
	/*****************************************************************************
	 *  Prediction
	 ****************************************************************************/
	
	// compute the time elapsed between the current and previous measurements in seconds
	double dt = abs(meas_package.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = meas_package.timestamp_;
	
	Prediction(dt);
	
	/*****************************************************************************
	 *  Update
	 ****************************************************************************/
	
	if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		UpdateRadar(meas_package);
		//cout << "radar update processed" << endl;
	} else if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) {
		UpdateLidar(meas_package);
		//cout << "lidar update processed" << endl;
	} else if (!use_radar_ && !use_laser_) {
		cout << "WARNING: both use_laser_ and use_rader_ are false, no update performed" << endl;
	} else if (!use_radar_) {
		cout << "WARNING: ignored radar measurement" << endl;
	} else if (!use_laser_) {
		cout << "WARNING: ignored lidar measurement" << endl;
	} else {
		cout << "ERROR: invalid meas_package.sensor_type_ during update" << endl;
	}
	
	//cout << "state vector:\n" << x_ << endl;
	//cout << "state covariance matrix:\n" << P_ << "\n" << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
	
	// create augmented state vector
	VectorXd x_aug(n_aug_);
	x_aug.head(n_x_) = x_;
	x_aug(n_x_) = 0;
	x_aug(n_x_ + 1) = 0;
	
	// create augmented state covariance
	MatrixXd P_aug(n_aug_, n_aug_);
	P_aug.fill(0.0);
	// set top left corner to P
	P_aug.topLeftCorner(n_x_, n_x_) = P_;
	// set bottom right corner to Q
	P_aug(n_aug_-2, n_aug_-2) = std_a_*std_a_;
	P_aug(n_aug_-1, n_aug_-1) = std_yawdd_*std_yawdd_;
	
	// create square root matrix
	MatrixXd P_aug_sqrt = P_aug.llt().matrixL();
	
	// create augmented sigma point matrix
	MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_+1);
	Xsig_aug.col(0) = x_aug;
	double c1 = sqrt(lambda_ + n_aug_);
	for (int i = 0; i < n_aug_; ++i) {
		Xsig_aug.col(i+1) = x_aug + c1*P_aug_sqrt.col(i);
		Xsig_aug.col(i+1+n_aug_) = x_aug - c1*P_aug_sqrt.col(i);
	}
	
	// create sigma point prediction matrix
	Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

	// process each augmented sigma point through process model function f(x,v)
	// predict sigma points in state space
	double delta_t2 = delta_t * delta_t;
	for (int i = 0; i < (2*n_aug_+1); ++i) {
		
		double p_x = Xsig_aug(0, i);
		double p_y = Xsig_aug(1, i);
		double v = Xsig_aug(2, i);
		double psi = Xsig_aug(3, i);
		double psi_dot = Xsig_aug(4, i);
		double v_a = Xsig_aug(5, i);
		double v_yawdd = Xsig_aug(6, i);
		
		// predict sigma point
		// avoid division by zero
		VectorXd pred(5);
		if (fabs(psi_dot) < 0.0001) {
			pred(0) = p_x + v*cos(psi)*delta_t + 0.5*delta_t2*cos(psi)*v_a;
			pred(1) = p_y + v*sin(psi)*delta_t + 0.5*delta_t2*sin(psi)*v_a;
		} else {
			pred(0) = p_x + (v/psi_dot)*(sin(psi+psi_dot*delta_t) - sin(psi)) + 0.5*delta_t2*cos(psi)*v_a;
			pred(1) = p_y + (v/psi_dot)*(-cos(psi+psi_dot*delta_t) + cos(psi)) + 0.5*delta_t2*sin(psi)*v_a;
		}
		pred(2) = v + delta_t*v_a;
		pred(3) = psi + psi_dot*delta_t + 0.5*delta_t2*v_yawdd;
		pred(4) = psi_dot + delta_t*v_yawdd;
		
		// write predicted sigma point into right column
		Xsig_pred_.col(i) = pred;
	}
	
	// predict state mean
	x_.fill(0.0);
	for (int i = 0; i < 2*n_aug_+1; ++i) {
		x_ += weights_(i)*Xsig_pred_.col(i);
	}
	
	// predict state covariance matrix
	P_.fill(0.0);
	for (int i = 0; i < 2*n_aug_+1; ++i) {
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		// angle normalization
		x_diff(3) = fmod(x_diff(3), 2.0 * M_PI);
		if (x_diff(3) > M_PI) {
			x_diff(3) -= 2.0 * M_PI;
		} else if (x_diff(3) < -M_PI) {
			x_diff(3) += 2.0 * M_PI;
		}
		P_ += weights_(i) * x_diff * x_diff.transpose();
	}
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
	
	// define measurement space dimensions
	int n_z = 2;
	
	// create matrix for sigma points in measurement space
	MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
	// transform sigma points into measurement space
	for (int i = 0; i < 2*n_aug_+1; ++i) {
		Zsig.col(i) << 	Xsig_pred_(0, i), // px
										Xsig_pred_(1, i); // py
	}
	
	// calculate mean predicted measurement
	VectorXd z_pred = VectorXd(n_z);
	z_pred.fill(0.0);
	for (int i = 0; i < 2*n_aug_+1; ++i) {
		z_pred += weights_(i)*Zsig.col(i);
	}
	
	// calculate measurement covariance matrix S
	MatrixXd S = MatrixXd(n_z,n_z);
	S.fill(0.0);
	for (int i = 0; i < 2*n_aug_+1; ++i) {
		VectorXd z_diff = Zsig.col(i) - z_pred;
		S += weights_(i) * z_diff * z_diff.transpose();
	}
	
	// create measurement noise matrix R, add to S
	MatrixXd R(n_z, n_z);
	R.fill(0.0);
	R(0, 0) = std_laspx_ * std_laspx_;
	R(1, 1) = std_laspy_ * std_laspy_;
	S += R;
	
	// calculate cross correlation matrix Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z);
	Tc.fill(0.0);
	for (int i = 0; i < 2*n_aug_+1; ++i) {
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		// angle normalization
		x_diff(3) = fmod(x_diff(3), 2.0 * M_PI);
		if (x_diff(3) > M_PI) {
			x_diff(3) -= 2.0 * M_PI;
		} else if (x_diff(3) < -M_PI) {
			x_diff(3) += 2.0 * M_PI;
		}
		
		VectorXd z_diff = Zsig.col(i) - z_pred;
		
		Tc += weights_(i)*x_diff*z_diff.transpose();
	}
	
	// calculate Kalman gain K
	MatrixXd Sinv = S.inverse();
	MatrixXd K = Tc*Sinv;
	
	//update state mean and covariance matrix
	VectorXd z_k1(n_z);
	z_k1 << meas_package.raw_measurements_[0], // measured px
					meas_package.raw_measurements_[1]; // measured py
	VectorXd z_k1_diff = z_k1 - z_pred;
	
	// calculate NIS value
	NIS_ = z_k1_diff.transpose() * Sinv * z_k1_diff;
	
	// update state vector and covariance matrix
	x_ += K*z_k1_diff;
	// angle normalization
	x_(3) = fmod(x_(3), 2.0 * M_PI);
	if (x_(3) > M_PI) {
		x_(3) -= 2.0 * M_PI;
	} else if (x_(3) < -M_PI) {
		x_(3) += 2.0 * M_PI;
	}
	P_ -= K*S*K.transpose();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
	
	// define measurement space dimensions
	int n_z = 3;
	
	// create matrix for sigma points in measurement space
	MatrixXd Zsig = MatrixXd(n_z, 2*n_aug_+1);
	// transform sigma points into measurement space
	for (int i = 0; i < 2*n_aug_+1; ++i) {
		double p_x = Xsig_pred_(0, i);
		double p_y = Xsig_pred_(1, i);
		double v = Xsig_pred_(2, i);
		double yaw = Xsig_pred_(3, i);
		//double yawd = Xsig_pred_(4, i);
		
		double rho = sqrt(p_x*p_x + p_y*p_y);
		double phi = atan2(p_y, p_x);
		// angle normalization
		phi = fmod(phi, 2.0 * M_PI);
		if (phi > M_PI) {
			phi -= 2.0 * M_PI;
		} else if (phi < -M_PI) {
			phi += 2.0 * M_PI;
		}
		double rho_dot = v*(p_x*cos(yaw) + p_y*sin(yaw))/rho;
		
		Zsig.col(i) << rho, phi, rho_dot;
	}
	
	// calculate mean predicted measurement
	VectorXd z_pred = VectorXd(n_z);
	z_pred.fill(0.0);
	for (int i = 0; i < 2*n_aug_+1; ++i) {
		z_pred += weights_(i)*Zsig.col(i);
	}
	
	// calculate measurement covariance matrix S
	MatrixXd S = MatrixXd(n_z,n_z);
	S.fill(0.0);
	for (int i = 0; i < 2*n_aug_+1; ++i) {
		VectorXd z_diff = Zsig.col(i) - z_pred;
		z_diff(1) = fmod(z_diff(1), 2.0 * M_PI);
		if (z_diff(1) > M_PI) {
			z_diff(1) -= 2.0 * M_PI;
		} else if (z_diff(1) < -M_PI) {
			z_diff(1) += 2.0 * M_PI;
		}
		S += weights_(i) * z_diff * z_diff.transpose();
	}
	
	// create measurement noise matrix R, add to S
	MatrixXd R(n_z, n_z);
	R.fill(0.0);
	R(0, 0) = std_radr_ * std_radr_;
	R(1, 1) = std_radphi_ * std_radphi_;
	R(2, 2) = std_radrd_ * std_radrd_;
	S += R;
	
	// calculate cross correlation matrix Tc
	MatrixXd Tc = MatrixXd(n_x_, n_z);
	Tc.fill(0.0);
	for (int i = 0; i < 2*n_aug_+1; ++i) {
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		x_diff(3) = fmod(x_diff(3), 2.0 * M_PI);
		if (x_diff(3) > M_PI) {
			x_diff(3) -= 2.0 * M_PI;
		} else if (x_diff(3) < -M_PI) {
			x_diff(3) += 2.0 * M_PI;
		}
		
		VectorXd z_diff = Zsig.col(i) - z_pred;
		z_diff(1) = fmod(z_diff(1), 2.0 * M_PI);
		if (z_diff(1) > M_PI) {
			z_diff(1) -= 2.0 * M_PI;
		} else if (z_diff(1) < -M_PI) {
			z_diff(1) += 2.0 * M_PI;
		}
		
		Tc += weights_(i)*x_diff*z_diff.transpose();
	}
	
	// calculate Kalman gain K
	MatrixXd Sinv = S.inverse();
	MatrixXd K = Tc*Sinv;
	
	//update state mean and covariance matrix
	VectorXd z_k1(n_z);
	z_k1 << meas_package.raw_measurements_[0], // measured rho
					meas_package.raw_measurements_[1], // measured phi
					meas_package.raw_measurements_[2]; // measured phi dot
	VectorXd z_k1_diff = z_k1 - z_pred;
	z_k1_diff(1) = fmod(z_k1_diff(1), 2.0 * M_PI);
	if (z_k1_diff(1) > M_PI) {
		z_k1_diff(1) -= 2.0 * M_PI;
	} else if (z_k1_diff(1) < -M_PI) {
		z_k1_diff(1) += 2.0 * M_PI;
	}
	
	// calculate NIS value
	NIS_ = z_k1_diff.transpose() * Sinv * z_k1_diff;
	
	// update state vector and covariance matrix
	x_ += K*z_k1_diff;
	// angle normalization
	x_(3) = fmod(x_(3), 2.0 * M_PI);
	if (x_(3) > M_PI) {
		x_(3) -= 2.0 * M_PI;
	} else if (x_(3) < -M_PI) {
		x_(3) += 2.0 * M_PI;
	}
	P_ -= K*S*K.transpose();
}
