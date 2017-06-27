#include "kalman_filter.h"
#include "Eigen/Dense"
#include <math.h>
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
//using std::vector;

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
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  std::cout << "EKF: Update Step->Laser->Update()" << std::endl;
  //std::cout << "z: " << z << std::endl;
    
  VectorXd z_pred = H_ * x_;
  //std::cout << "z_pred: " << z_pred << std::endl;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  //std::cout << "EKF: Update Step->Radar->Update->New estimate" << std::endl;
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  std::cout << "EKF: Update Step->Radar->EKFUpdate()" << std::endl;
  //std::cout << "UpdateEKF () z:" << z << std::endl;
    
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];   
  
  //float PI = 3.14159265359;
  float rho = sqrt(px*px + py*py);
  float theta = atan2(py,px);
  float rhodot = (px*vx + py*vy)/rho;
  
  if(theta > M_PI)
    {
        theta = theta - 2.*M_PI;
    }
  else if(theta < (-1.*M_PI))
    {
        theta = theta + 2.*M_PI;
    }
    
  //while (theta> M_PI) {theta -=2.*M_PI;}
  //while (theta< -1*M_PI) {theta +=2.*M_PI;}
  
  //std::cout << "UpdateEKF () rho:" << rho << std::endl;
  //std::cout << "UpdateEKF () theta:" << theta << std::endl;
  //std::cout << "UpdateEKF () rhodot:" << rhodot << std::endl;
    
  //check division by zero
  if(std::abs(rho) < 0.0001){
      std::cout << "UpdateEKF () - Error - Division by Zero" << std::endl;
	return ;
  }
  
  VectorXd z_pred;
  z_pred = VectorXd(3);
  z_pred << rho, theta, rhodot;
  std::cout << "EKF: Update Step->Radar->EKFUpdate() theta:" << theta << std::endl;

  //VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  //std::cout << "UpdateEKF () y:" << y << std::endl;
  MatrixXd Ht = H_.transpose();
  //std::cout << "UpdateEKF () Ht:" << Ht << std::endl;
  MatrixXd S = H_ * P_ * Ht + R_;
  //std::cout << "UpdateEKF () S:" << S << std::endl;
  MatrixXd Si = S.inverse();
  //std::cout << "UpdateEKF () Si:" << Si << std::endl;
  MatrixXd PHt = P_ * Ht;
  //std::cout << "UpdateEKF () PHt:" << PHt << std::endl;
  MatrixXd K = PHt * Si;
  //std::cout << "UpdateEKF () K:" << K << std::endl;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
