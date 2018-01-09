##FusionEKF.cpp

In FusionEKF.cpp, we have given some starter code for implementing sensor fusion. In this file, you won't need to include the actual Kalman filter equations; instead, you will be initializing variables, initializing the Kalman filters, and then calling functions that implement the prediction step or update step. You will see TODO comments indicating where to put your code.

You will need to:

    initialize variables and matrices (x, F, H_laser, H_jacobian, P, etc.)
    initialize the Kalman filter position vector with the first sensor measurements
    modify the F and Q matrices prior to the prediction step based on the elapsed time between measurements
    call the update step for either the lidar or radar sensor measurement. Because the update step for lidar and radar are slightly different, there are different functions for updating lidar and radar.

##Initializing the Kalman Filter in FusionEKF.cpp

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

##Predict and Update Steps in FusionEKF.cpp

Once the Kalman filter gets initialized, the next iterations of the for loop will call the ProcessMeasurement() function to do the predict and update steps.

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
  } else {
    // Laser updates
  }

In FusionEKF.cpp, you will see references to a variable called ekf_. The ekf_ variable is an instance of the KalmanFilter class. You will use ekf_ to store your Kalman filter variables (x, P, F, H, R, Q) and call the predict and update functions. Let's talk more about the KalmanFilter class.


##KalmanFilter Class

kalman_filter.h defines the KalmanFilter class containing the x vector as well as the P, F, Q, H and R matrices. The KalmanFilter class also contains functions for the prediction step as well as the Kalman filter update step (lidar) and extended Kalman filter update step (radar).

You will need to add your code to kalman_filter.cpp to implement the prediction and update equations. You do not need to modify 'kalman_filter.h'.

Because lidar uses linear equations, the update step will use the basic Kalman filter equations. On the other hand, radar uses non-linear equations, so the update step involves linearizing the equations with the Jacobian matrix. The Update function will use the standard Kalman filter equations. The UpdateEKF will use the extended Kalman filter equations:

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}


##Tools.cpp

This file is relatively straight forward. You will implement functions to calculate root mean squared error and the Jacobian matrix:

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
}
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}

HINT: You implemented these already in the coding quizzes.


##Summary of What Needs to Be Done

####In tools.cpp, fill in the functions that calculate root mean squared error (RMSE) and the Jacobian matrix.
DONE

####Fill in the code in FusionEKF.cpp. You'll need to initialize the Kalman Filter, prepare the Q and F matrices for the prediction step, and call the radar and lidar update functions.
Initialize done, 

Q & F done,

TO-DO: Rader initialization


####In kalman_filter.cpp, fill out the Predict(), Update(), and UpdateEKF() functions.

Predict() DONE

Update() OPEN

UpdateEKF() OPEN