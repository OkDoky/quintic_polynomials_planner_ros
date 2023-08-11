#include "quintic_polynomials_planner_ros/quintic_polynomial.h"

QuinticPolynomialsPlanner::QuinticPolynomialsPlanner(double max_speed, double max_throttle, double min_point_resolution, double heading_yaw_error_threshold) 
    : max_speed_(max_speed), max_throttle_(max_throttle), speed_epsilon_(0.05),
    xy_goal_tolerance_(0.05), feedback_epsilon_(0.1), min_point_resolution_(min_point_resolution), heading_yaw_error_threshold_(heading_yaw_error_threshold){
  std::cout << "[QuinticPolynomialPlanner] init speed : " << max_speed_ << ", throttle : " << max_throttle_ << std::endl;
}

QuinticPolynomialsPlanner::~QuinticPolynomialsPlanner(){}

QuinticPolynomialsPlanner::QuinticPolynomialsPlanner()
    : max_speed_(0.0), max_throttle_(0.0), speed_epsilon_(0.05),
    xy_goal_tolerance_(0.05), feedback_epsilon_(0.1), min_point_resolution_(0.03), heading_yaw_error_threshold_(0.1745){
  std::cout << "[QuinticPolynomialPlanner] only init object.." << std::endl;
}

void QuinticPolynomialsPlanner::initialize(double max_speed, double max_throttle, double min_point_resolution, double heading_yaw_error_threshold){
  max_speed_ = max_speed;
  max_throttle_ = max_throttle;
  speed_epsilon_ = 0.05;
  xy_goal_tolerance_ = 0.5;
  feedback_epsilon_ = 0.1;
  max_time_ = 15.0;
  min_point_resolution_ = min_point_resolution;
  heading_yaw_error_threshold_ = heading_yaw_error_threshold;
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
  double v = feedback_vel.linear.x;
  if (abs(v) < 0.1){
    v = std::copysign(0.1, v);
  }
  // double decel_from_v_triang_area = triangle_area(v, max_throttle_);
  double decel_from_vm_triang_area = triangle_area(max_speed_, max_throttle_);
  double acc_trapezoid_area = trapezoid_area(v, max_speed_, max_throttle_);
  int i = 0;
  if(goal_to_end_plan <= xy_goal_tolerance_){
    // deceleration
    if (v + speed_epsilon_ >= max_speed_){
      // donot consider accel
      if (decel_from_vm_triang_area > dist_to_goal - xy_goal_tolerance_){
        T = v/max_throttle_;
      } else{
        T = (dist_to_goal - decel_from_vm_triang_area)/max_speed_ + max_speed_/max_throttle_;
      }
    }else{
      // consider accel part
      if (decel_from_vm_triang_area + acc_trapezoid_area > dist_to_goal - xy_goal_tolerance_){
        T = max_speed_/max_throttle_ + (dist_to_goal-decel_from_vm_triang_area)/((max_speed_+v)/2);
      } else {
        // accel -> tracking -> decel
        T = max_speed_/max_throttle_ + abs(max_speed_- v)/max_throttle_ + (dist_to_goal - decel_from_vm_triang_area - acc_trapezoid_area)/max_speed_;
      }
    }
  }else{
    // acceleration
    if (v <= max_speed_-speed_epsilon_){
      T = (abs(max_speed_ - v)/max_throttle_) + ((dist_to_goal - acc_trapezoid_area)/max_speed_);
    } else{
      T = dist_to_goal/v;
    }
  }
  if (T > max_time_){
    T = max_time_;
    std::cout << "[Quintic] caltimer check " << i << ", T : " << T << ", dist : " << dist_to_goal << std::endl;
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
  double hyaw;
  if (!free_target_vector){
    tyaw = atan2(global_plan.back().pose.position.y -global_plan[0].pose.position.y,
                global_plan.back().pose.position.x -global_plan[0].pose.position.x);
  }else{
    tyaw = atan2(goal_pose.pose.position.y-global_pose.pose.position.y,
                goal_pose.pose.position.x-global_pose.pose.position.x);
  }
  tvx = cos(tyaw);
  tvy = sin(tyaw);
  
  hyaw = atan2(tpy-cpy, tpx-cpx);

  cyaw = tf2::getYaw(global_pose.pose.orientation);
  if (abs(cyaw - hyaw) > heading_yaw_error_threshold_){
    cyaw = hyaw;
  } // 10 degrees
  if (abs(feedback_vel.linear.x) < abs(feedback_epsilon_)){
    double fe = std::copysign(feedback_epsilon_, feedback_vel.linear.x);
    cvx = (feedback_vel.linear.x + fe) * cos(cyaw);
    cvy = (feedback_vel.linear.x + fe) * sin(cyaw);
  }else{
    cvx = feedback_vel.linear.x * cos(cyaw);
    cvy = feedback_vel.linear.x * sin(cyaw);
  }
  
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
  int num_point = static_cast<int>(T*50); // 10 is hz of local planner..
  const int degree = 5;
  double x,y;
  std::vector<geometry_msgs::PoseStamped> polynomial_plan;
  for (int i = 0; i <= num_point; i++){
    x = outputPolynomial(i*0.02, coefficients_x);
    y = outputPolynomial(i*0.02, coefficients_y);
    if (std::isnan(x) || std::isnan(y)){
      std::cout << "[quintic] nan value, T is : " << T << ", v : " << feedback_vel.linear.x << std::endl;
    }
    if (hypot(temp_pose.pose.position.x - x, temp_pose.pose.position.y - y) < min_point_resolution_){
      continue;
    }
    temp_pose.pose.position.x = x;
    temp_pose.pose.position.y = y;
    polynomial_plan.push_back(temp_pose);
  }
  polynomial_plan.push_back(goal_pose);
  if (polynomial_plan.size() < degree +2){ // spline has to get fixed more then degree points..
    tf2::Quaternion q;
    q.setRPY(0,0,0);
    smooth_plan.push_back(polynomial_plan.front());
    for (int j = 1; j < polynomial_plan.size(); j++){
      smooth_plan.push_back(polynomial_plan[j]);
      double dx = smooth_plan[j].pose.position.x - smooth_plan[j-1].pose.position.x;
      double dy = smooth_plan[j].pose.position.y - smooth_plan[j-1].pose.position.y;
      double yaw = atan2(dy, dx);
      q.setRPY(0,0,yaw);
      smooth_plan[j-1].pose.orientation = tf2::toMsg(q);
    }
  }else{
    splineSmooth(polynomial_plan, smooth_plan, degree);
  }
}

