#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <Eigen/Eigen>
#include "quintic_polynomials_planner_ros/GetPolynomials.h"

using namespace std;
using Eigen::MatrixXd;

class Quintic_Polynomials_Planner {
  private:
    vector<double> coefficients_x;
    vector<double> coefficients_y;

  public:
    std::vector<std_msgs::Float32> convertToFloat32Array(const std::vector<double>& input);
    vector<double> get_coefficients(vector<double> start, vector<double> end, double T);
    bool solve_polynomials(quintic_polynomials_planner_ros::GetPolynomials::Request& req, quintic_polynomials_planner_ros::GetPolynomials::Response& res);
};

std::vector<std_msgs::Float32> Quintic_Polynomials_Planner::convertToFloat32Array(const std::vector<double>& input) {
  std::vector<std_msgs::Float32> output;
  output.resize(input.size());

  for (size_t i = 0; i < input.size(); ++i) {
    output[i].data = static_cast<float>(input[i]);
  }

  return output;
}

vector<double> Quintic_Polynomials_Planner::get_coefficients(vector<double> start, vector<double> end, double T) {
  MatrixXd A = MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
			    3*T*T, 4*T*T*T,5*T*T*T*T,
			    6*T, 12*T*T, 20*T*T*T;
		
	MatrixXd B = MatrixXd(3,1);	    
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
			    end[1]-(start[1]+start[2]*T),
			    end[2]-start[2];
			    
	MatrixXd Ai = A.inverse();
	
	MatrixXd C = Ai*B;
	
	vector<double> result = {start[0], start[1], .5*start[2]};
	for(int i = 0; i < C.size(); i++)
	{
	    result.push_back(C.data()[i]);
	}
	
    return result;
}

bool Quintic_Polynomials_Planner::solve_polynomials(quintic_polynomials_planner_ros::GetPolynomials::Request& req, quintic_polynomials_planner_ros::GetPolynomials::Response& res) {
  vector<double> start_x = {req.start_pnt.x, req.start_vel.linear.x, req.start_acc.linear.x};
  vector<double> end_x = {req.goal_pnt.x, req.goal_vel.linear.x, req.goal_acc.linear.x};
  vector<double> start_y = {req.start_pnt.y, req.start_vel.linear.y, req.start_acc.linear.x};
  vector<double> end_y = {req.goal_pnt.y, req.goal_vel.linear.y, req.goal_acc.linear.y};
  double T = req.T.data;

  coefficients_x = get_coefficients(start_x, end_x, T);
  coefficients_y = get_coefficients(start_y, end_y, T);

  res.x_coeff = convertToFloat32Array(coefficients_x);
  res.y_coeff = convertToFloat32Array(coefficients_y);

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "quintic_polynomials_server");
  ros::NodeHandle nh;
  Quintic_Polynomials_Planner planner;
  ros::ServiceServer server = nh.advertiseService("get_polynomials", &Quintic_Polynomials_Planner::solve_polynomials, &planner);
  ROS_INFO("Quintic Polynomials Planner is ready.");
  ros::spin();

  return 0;
}
