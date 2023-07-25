#include "quintic_polynomials_planner_ros/quintic_polynomial.h"

QuinticPolynomialsPlanner::QuinticPolynomialsPlanner(double max_speed, double max_throttle) 
    : max_speed_(max_speed), max_throttle_(max_throttle), speed_epsilon_(0.5), 
    xy_goal_tolerance_(0.05), feedback_epsilon_(0.05){
  std::cout << "[QuinticPolynomialPlanner] init speed : " << max_speed_ << ", throttle : " << max_throttle_ << std::endl;
}

QuinticPolynomialsPlanner::~QuinticPolynomialsPlanner(){}

QuinticPolynomialsPlanner::QuinticPolynomialsPlanner()
    : max_speed_(0.0), max_throttle_(0.0), speed_epsilon_(0.5), 
    xy_goal_tolerance_(0.05), feedback_epsilon_(0.05){
  std::cout << "[QuinticPolynomialPlanner] only init object.." << std::endl;
}

void QuinticPolynomialsPlanner::initialize(double max_speed, double max_throttle){
  max_speed_ = max_speed;
  max_throttle_ = max_throttle;
  speed_epsilon_ = 0.5;
  xy_goal_tolerance_ = 0.5;
  feedback_epsilon_ = 0.05;
  max_time_ = 15.0;
  std::cout << "[QuinticPolynomialsPlanner] init speed : " << max_speed_ <<", throttle : " << max_throttle_ << std::endl;
}

void QuinticPolynomialsPlanner::calculateTrackingTime(const geometry_msgs::PoseStamped& global_pose,
    const geometry_msgs::PoseStamped& goal_pose,
    const geometry_msgs::Twist& feedback_vel,
    const std::vector<geometry_msgs::PoseStamped>& global_plan,
    double& T){
  double dist_to_goal = hypot(global_pose.pose.position.x - goal_pose.pose.position.x,
                              global_pose.pose.position.y - goal_pose.pose.position.y);
  double goal_to_end_plan = hypot(goal_pose.pose.position.x - global_plan.back().pose.position.x,
                                  goal_pose.pose.position.y - global_plan.back().pose.position.y);
  double pp_dist = calPerpendicularDistance(global_plan[0].pose.position.x, global_plan[0].pose.position.y,
      global_plan.back().pose.position.x, global_plan.back().pose.position.y,
      global_pose.pose.position.x, global_pose.pose.position.y,
      goal_pose.pose.position.x, goal_pose.pose.position.y);
  double v = feedback_vel.linear.x;
  if(goal_to_end_plan <= xy_goal_tolerance_){
    // deceleration
    if (abs(v) < 0.1){
        T = 2 * sqrt(dist_to_goal);
    }else{
      T = ((dist_to_goal - (1/2)*pow(v, 2)/max_throttle_)/v) + (v/max_throttle_);
    }
  }else{
    // acceleration
    if (v <= max_speed_-speed_epsilon_){
      T = ((max_speed_ - v)/max_throttle_) + ((dist_to_goal - (pow(max_speed_,2)-pow(v,2))/(2*max_throttle_))/max_speed_);
    } else{
      if (abs(v) < 0.001){
        v = 0.001;
      }
      T = dist_to_goal/v;
    }
  }
  if (T > max_time_){
    T = max_time_;
  }
}

void QuinticPolynomialsPlanner::getPolynomialPlan(const geometry_msgs::PoseStamped& global_pose,
    const geometry_msgs::PoseStamped& goal_pose,
    const geometry_msgs::Twist& feedback_vel,
    const std::vector<geometry_msgs::PoseStamped>& global_plan,
    const std::string& frame_id,
    const bool free_target_vector,
    std::vector<geometry_msgs::PoseStamped>& smooth_plan){

  if (global_plan.size() <= 1){
    smooth_plan.push_back(goal_pose);
    return;
  }
  double cpx, cpy, cvx, cvy, cax, cay;
  double tpx, tpy, tvx, tvy, tax, tay;
  cpx = global_pose.pose.position.x;
  cpy = global_pose.pose.position.y;
  tpx = goal_pose.pose.position.x;
  tpy = goal_pose.pose.position.y;

  double cyaw, tyaw;
  cyaw = tf2::getYaw(global_pose.pose.orientation);
  cvx = (feedback_vel.linear.x + feedback_epsilon_) * cos(cyaw);
  cvy = (feedback_vel.linear.x + feedback_epsilon_) * sin(cyaw);

  if (!free_target_vector){
    tyaw = atan2(global_plan.back().pose.position.y -global_plan[0].pose.position.y,
                global_plan.back().pose.position.x -global_plan[0].pose.position.x);
  }else{
    tyaw = atan2(goal_pose.pose.position.y-global_pose.pose.position.y,
                goal_pose.pose.position.x-global_pose.pose.position.x);
  }
  tvx = cos(tyaw);
  tvy = sin(tyaw);
  tax = 0.0;
  tay = 0.0;
  cax = 0.0;
  cay = 0.0;

  vector<double> start_x = {cpx, cvx, cax};
  vector<double> end_x = {tpx, tvx, tax};
  vector<double> start_y = {cpy, cvy, cay};
  vector<double> end_y = {tpy, tvy, tay};
  
  double T;
  calculateTrackingTime(global_pose, goal_pose, feedback_vel, global_plan, T);
  vector<double> coefficients_x = getCoefficients(start_x, end_x, T);
  vector<double> coefficients_y = getCoefficients(start_y, end_y, T);

  geometry_msgs::PoseStamped temp_pose;
  tf2::Quaternion temp_quat;
  temp_pose.header.frame_id = frame_id;
  temp_pose.header.stamp = ros::Time::now();
  int num_point = static_cast<int>(T*10); // 10 is hz of local planner..
  double x,y;
  std::vector<geometry_msgs::PoseStamped> polynomial_plan;
  for (int i = 1; i <= num_point; i++){
    x = outputPolynomial(i*0.1, coefficients_x);
    y = outputPolynomial(i*0.1, coefficients_y);
    temp_pose.pose.position.x = x;
    temp_pose.pose.position.y = y;
    polynomial_plan.push_back(temp_pose);
  }
  splineSmooth(polynomial_plan, smooth_plan);

}

