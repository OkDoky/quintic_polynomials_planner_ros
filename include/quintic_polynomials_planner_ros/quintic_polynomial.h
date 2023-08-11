#ifndef QUINTIC_POLYNOMIAL_H
#define QUINTIC_POLYNOMIAL_H

#include <iostream>
#include <fstream>
#include <numeric>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "spline_smooth_filter.h"
#include <fenv.h>

using namespace std;
using Eigen::MatrixXd;


vector<double> getCoefficients(vector<double> start, vector<double> end, double T) {
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

inline double trapezoid_area(double v1, double v2, double slope){
  return (pow(std::max(v1,v2),2) - pow(std::min(v1,v2),2))/(2*slope);
}

inline double triangle_area(double v, double slope){
  return (pow(v,2)/(2*slope));
}

inline double outputPolynomial(double time, vector<double> coeffs){
  if (std::isnan(time) || std::isinf(time)){
    std::cout << "[outputPolynomial] time is nan or inf : " << time << std::endl;
  }
  for (double coeff : coeffs){
    if (std::isnan(coeff)){
      std::cout << "[outputPolynomial] coeff is nan.." << std::endl;
    }
  }
  std::vector<int> indices(coeffs.size());
  std::iota(indices.begin(), indices.end(), 0);  // Fill with 0, 1, 2, ..., coeffs.size()-1

  double result_ = std::inner_product(
    indices.begin(), indices.end(), coeffs.begin(), 0.0,
    std::plus<double>(),  // Combining function
    [&time](int i, double coeff){ return pow(time,i) * coeff; });  // Product function
  return result_;
}

class QuinticPolynomialsPlanner {
  private:
    double max_speed_, max_throttle_;
    double speed_epsilon_;
    double feedback_epsilon_;
    double xy_goal_tolerance_;
    double max_time_;
    double min_point_resolution_;
    double heading_yaw_error_threshold_;
  public:
    QuinticPolynomialsPlanner();
    QuinticPolynomialsPlanner(double max_speed, double max_throttle, double min_point_resolution, double heading_yaw_error_threshold);
    ~QuinticPolynomialsPlanner();
    void initialize(double max_speed, double max_throttle, double min_point_resolution, double heading_yaw_error_threshold);
    void calculateTrackingTime(const geometry_msgs::PoseStamped& global_pose,
      const geometry_msgs::PoseStamped& goal_pose,
      const geometry_msgs::Twist& feedback_vel,
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      double& T);
    void getPolynomialPlan(const geometry_msgs::PoseStamped& global_pose,
      const geometry_msgs::PoseStamped& goal_pose,
      const geometry_msgs::Twist& feedback_vel,
      const std::vector<geometry_msgs::PoseStamped>& global_plan,
      const std::string& frame_id,
      const bool free_target_vector,
      std::vector<geometry_msgs::PoseStamped>& smooth_plan);
};


#endif /* QUINTIC_POLYNOMIAL_H */
