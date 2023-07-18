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

using namespace std;
using Eigen::MatrixXd;

/**
 * @brief perpendicular distance for line and two points.
 * 
 * @param x1 global plan first point x
 * @param y1 global plan first point y
 * @param x2 global plan end point x
 * @param y2 global plan end point y
 * @param x0 global pose x
 * @param y0 global pose y
 * @param x3 goal pose x
 * @param y3 goal pose y
 * @return double 
 */
double calPerpendicularDistance(double x1, double y1, double x2, double y2, 
    double x0, double y0, double x3, double y3){
  double signed_dist_0 = ((x2 - x1) * (y0 - y1) - (x0 - x1) * (y2 - y1))/
      sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  double signed_dist_3 = ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1))/ 
      sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  if (signed_dist_0 * signed_dist_3 < 0){
    return abs(signed_dist_0) + abs(signed_dist_3);
  }else{
    return abs(signed_dist_0 - signed_dist_3);
  }
}

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

inline double outputPolynomial(double time, vector<double> coeffs){
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
  public:
    QuinticPolynomialsPlanner();
    QuinticPolynomialsPlanner(double max_speed, double max_throttle);
    ~QuinticPolynomialsPlanner();
    void initialize(double max_speed, double max_throttle);
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
