#include <iostream>
#include "multi_object_tracker/tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	VectorXd rmse(estimations[0].size());
	rmse.fill(0.0);
	
	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() == 0) {
		cout << "Error: estimations vector is empty" << endl;
		return rmse;
	}
	if (estimations.size() != ground_truth.size()) {
		cout << "Error: estimations and ground_truth vectors are not the same size" << endl;
		return rmse;
	}
	
	// accumulate squared residuals
	VectorXd squared_sum;
	squared_sum.setZero(estimations[0].size());
	for(int i=0; i < estimations.size(); ++i){
		VectorXd res = estimations[i] - ground_truth[i];
		squared_sum = squared_sum.array() + res.array().pow(2);
	}
	
	// calculate the mean
	VectorXd mean = squared_sum.array() / estimations.size();
	
	// calculate the squared root
	rmse = mean.array().sqrt();
	
  return rmse;
}
