#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd(5);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 30;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 30;

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

    /**
    DONE:

    Complete the initialization. See ukf.h for other member properties.

    Hint: one or more values initialized above might be wildly off...
    */
    // the current NIS for radar
    NIS_radar_ = 0.0;

    // the current NIS for laser
    NIS_laser_ = 0.0;

    is_initialized_ = false;

    n_x_ = 5;
    n_aug_ = 7;
    lambda_ = 3 - n_aug_;

    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

    weights_.fill(0.5 * (lambda_ + n_aug_));
    weights_(0) = lambda_ / (lambda_ + n_aug_);

    int n_z = 2;
    R_laser_ = MatrixXd(n_z, n_z);
    R_laser_.fill(0.0);
    R_laser_(0, 0) = std_laspx_ * std_laspx_;
    R_laser_(1, 1) = std_laspy_ * std_laspy_;

    n_z = 3;
    R_radar_ = MatrixXd(n_z, n_z);
    R_radar_.fill(0.0);
    R_radar_(0, 0) = std_radr_ * std_radr_;
    R_radar_(1, 1) = std_radphi_ * std_radphi_;
    R_radar_(2, 2) = std_radrd_ * std_radrd_;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /**
    DONE:

    Complete this function! Make sure you switch between lidar and radar
    measurements.
    */
    cout << "Start ProcessMeasurement()" << endl;
    if (!is_initialized_) {
        P_ = MatrixXd::Identity(n_x_, n_x_);

        x_.fill(0.0);

        if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            double px = meas_package.raw_measurements_[0];
            double py = meas_package.raw_measurements_[1];
            x_ << px, py, 0, 0, 0;
        } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            //Convert radar from polar to cartesian coordinates
            double rho, phi, rho_dot;
            rho = meas_package.raw_measurements_[0];
            phi = meas_package.raw_measurements_[1];
            rho_dot = meas_package.raw_measurements_[2];
            x_ << rho * cos(phi), rho * sin(phi), 0, 0, 0;
        }

        time_us_ = meas_package.timestamp_;
        // done initializing, no need to predict or update
        is_initialized_ = true;

        return;
    }

    float delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0; // delta_t in seconds
    cout << "delta_t: " << delta_t << endl;
    time_us_ = meas_package.timestamp_;

    Prediction(delta_t);
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        UpdateLidar(meas_package);
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        UpdateRadar(meas_package);
    }
    cout << "End ProcessMeasurement()" << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    /**
    DONE:

    Complete this function! Estimate the object's location. Modify the state
    vector, x_. Predict sigma points, the state, and the state covariance matrix.
    */
    cout << "Start Prediction()" << endl;

    Tools tools;

    // Generate augmented sigma points
    VectorXd x_aug = VectorXd(n_aug_);//Create augmented mean vector
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);//Create augmented state covariance

    x_aug.fill(0.0);
    P_aug.fill(0.0);

    //Create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
    Xsig_aug.fill(0.0);


    //Create augmented mean state
    x_aug.head(n_x_) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;

    //Create augmented covariance matrix
    MatrixXd Q = MatrixXd(2, 2);
    Q << std_a_ * std_a_, 0,
            0, std_yawdd_ * std_yawdd_;

    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug.bottomRightCorner(2, 2) = Q;

    //Create square root matrix
    MatrixXd P_aug_sqrt = P_aug.llt().matrixL();

    //Create augmented sigma points
    Xsig_aug.col(0) = x_aug;
    for (int i = 0; i < n_aug_; i++) {
        Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * P_aug_sqrt.col(i);
        Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * P_aug_sqrt.col(i);
    }

    //Predict state by predicting new new Xsig_pred


    VectorXd change = VectorXd(5);
    VectorXd new_state = VectorXd(5);
    for (int i = 0; i < Xsig_pred_.cols(); i++) {
        VectorXd curr_sigma_point = Xsig_aug.col(i);
        VectorXd curr_state = curr_sigma_point.head(5);

        double v = curr_sigma_point(2);
        double psi = curr_sigma_point(3); // yaw
        double psi_dot = curr_sigma_point(4); // yaw dot
        double nu_a = curr_sigma_point(5);
        double nu_psi_dot_dot = curr_sigma_point(6);

        double delta_t_pow = delta_t * delta_t;

        VectorXd noise = VectorXd(5);
        noise(0) = 0.5 * delta_t_pow * cos(psi) * nu_a;
        noise(1) = 0.5 * delta_t_pow * sin(psi) * nu_a;
        noise(2) = delta_t * nu_a;
        noise(3) = 0.5 * delta_t_pow * nu_psi_dot_dot;
        noise(4) = delta_t * nu_psi_dot_dot;

        if (psi_dot == 0) {
            change(0) = v * cos(psi) * delta_t;
            change(1) = v * sin(psi) * delta_t;
            change(2) = 0;
            change(3) = psi_dot * delta_t;
            change(4) = 0;
        } else {
            double v_psi = v / psi_dot;
            double new_psi = psi + psi_dot * delta_t;
            change(0) = v_psi * (sin(new_psi) - sin(psi));
            change(1) = v_psi * (-cos(new_psi) + cos(psi));
            change(2) = 0;
            change(3) = psi_dot * delta_t;
            change(4) = 0;
        }
        new_state = curr_state + change + noise;
        Xsig_pred_.col(i) = new_state;
    }

    // Reset and predict state mean and state covariance matrix
    x_.fill(0.0);
    P_.fill(0.0);


    //predicted state mean
    x_ = Xsig_pred_ * weights_;

    //predict state covariance matrix
    for (int j = 0; j < Xsig_pred_.cols(); j++) {
        VectorXd x_diff = Xsig_pred_.col(j) - x_;

        tools.NormalizeAngle(x_diff(3));

        P_ += weights_(j) * x_diff * x_diff.transpose();
    }

    cout << "x" << endl << x_ << endl;
    cout << "P" << endl << P_ << endl;

    cout << "End Prediction()" << endl;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
    DONE:

    Complete this function! Use lidar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the lidar NIS.
    */

    //create matrix for sigma points in measurement space
    MatrixXd Zsig = Xsig_pred_.block(0, 0, n_z, 2 * n_aug_ + 1);

    Update(Zsig, R_laser_, meas_package);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
    DONE:

    Complete this function! Use radar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the radar NIS.
    */
    int n_z = 3;

    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    Zsig.fill(0.0);

    double p_x;
    double p_y;
    double v;
    double psi;
    double psi_dot;
    double sqrt_p_sum;
    //transform sigma points into measurement space
    for (int i = 0; i < Xsig_pred_.cols(); i++) {
        p_x = Xsig_pred_(0, i);
        p_y = Xsig_pred_(1, i);
        v = Xsig_pred_(2, i);
        psi = Xsig_pred_(3, i);
        psi_dot = Xsig_pred_(4, i);

        sqrt_p_sum = sqrt(p_x * p_x + p_y * p_y);

        Zsig(0, i) = sqrt_p_sum;
        Zsig(1, i) = atan2(p_y, p_x);
        Zsig(2, i) = ((p_x * cos(psi) * v) + (p_y * sin(psi) * v)) / sqrt_p_sum;
    }


    Update(Zsig, R_radar_, meas_package);
}

void UKF::Update(MatrixXd Zsig, MatrixXd R, MeasurementPackage meas_package) {
    Tools tools;
    int n_z = Zsig.rows();

    //Measurement covariance matrix
    MatrixXd S = MatrixXd(n_z, n_z);
    S.fill(0.0);

    //Mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred.fill(0.0);

    //calculate mean predicted measurement
    for (int i = 0; i < Zsig.cols(); i++) {
        z_pred += weights_(i) * Zsig.col(i);
    }

    for (int i = 0; i < Zsig.cols(); i++) {
        VectorXd z_diff = VectorXd(n_z);
        z_diff = Zsig.col(i) - z_pred;

        tools.NormalizeAngle(z_diff(1));

        S += weights_(i) * z_diff * z_diff.transpose();
    }

    S += R;

    //mesurements matrix
    VectorXd z = VectorXd(n_z);

    z << meas_package.raw_measurements_;

    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);

    Tc.fill(0.0);
    //calculate cross correlation matrix
    for (int i = 0; i < Zsig.cols(); i++) {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        tools.NormalizeAngle(x_diff(3));

        VectorXd z_diff = Zsig.col(i) - z_pred;
        tools.NormalizeAngle(z_diff(1));

        Tc += weights_(i) * x_diff * z_diff.transpose();
    }

    //calculate Kalman gain K;
    MatrixXd K = MatrixXd(n_x_, n_z);
    K = Tc * S.inverse();
    //update state mean and covariance matrix

    VectorXd z_diff = z - z_pred;
    tools.NormalizeAngle(z_diff(1));

    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();

    //calculate the Normalized Innovation Square
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
    } else {
        NIS_laser_ = z_diff.transpose() * S.inverse() * z_diff;
    }

}
