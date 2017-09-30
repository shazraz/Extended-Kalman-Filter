#include <iostream>
#include <cmath>
#include "tools.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  //Validate the estimations vector
  if(estimations.size() == 0 || estimations.size() != ground_truth.size()){
    cout<<"Error in size of Estimations vector or size mismatch with Ground Truth vector";
    return rmse;
  }
  
  //Accumulate the residual
  for(int i = 0; i < estimations.size(); ++i){
  VectorXd residual = estimations[i] - ground_truth[i];
  rmse = rmse + (residual.array() * residual.array()).matrix();
  }

  //Mean and Sqrt the error
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  //Initalize the Jacobian
  MatrixXd Hj(3,4);
  //Retrive the state values from the vector
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //Set the Jacobian to zero
  Hj << 0,0,0,0,
	0,0,0,0,
	0,0,0,0;

  //Check for small values of position magnitude to avoid division by zero
  float rho = pow((pow(px,2) + pow(py,2)), 0.5);
  if( rho < 0.0001){
    cout << "Value of rho too small - possible div by 0. Reassigning rho = 0.0005";
    rho = 0.0001;
  }

  float inv_rho = pow(rho, -1);
  Hj(0,0) = px * inv_rho;
  Hj(1,0) = -py * pow(inv_rho,2);
  Hj(2,0) = py * (vx*py - vy*px) * pow(inv_rho, 3);
  Hj(0,1) = py * inv_rho;
  Hj(1,1) = px * pow(inv_rho, 2);
  Hj(2,1) = px * (vy*px - vx*py) * pow(inv_rho,3);
  Hj(2,2) = Hj(0,0);
  Hj(2,3) = Hj(0,1);

  return Hj;

}
