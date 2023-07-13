#include <quintic_polynomials_planner_ros/quintic_polynomial.h>

ros::Publisher pub_polynomial_plan_;

std::vector<std_msgs::Float32> convertToFloat32Array(const std::vector<double>& input) {
  std::vector<std_msgs::Float32> output;
  output.resize(input.size());

  for (size_t i = 0; i < input.size(); ++i) {
    output[i].data = static_cast<float>(input[i]);
  }

  return output;
}

bool solvePolynomials(quintic_polynomials_planner_ros::GetPolynomials::Request& req, quintic_polynomials_planner_ros::GetPolynomials::Response& res) {
  vector<double> start_x = {req.start_pnt.x, req.start_vel.linear.x, req.start_acc.linear.x};
  vector<double> end_x = {req.goal_pnt.x, req.goal_vel.linear.x, req.goal_acc.linear.x};
  vector<double> start_y = {req.start_pnt.y, req.start_vel.linear.y, req.start_acc.linear.x};
  vector<double> end_y = {req.goal_pnt.y, req.goal_vel.linear.y, req.goal_acc.linear.y};
  double T = req.T.data;

  vector<double> coefficients_x = getCoefficients(start_x, end_x, T);
  vector<double> coefficients_y = getCoefficients(start_y, end_y, T);

  res.x_coeff = convertToFloat32Array(coefficients_x);
  res.y_coeff = convertToFloat32Array(coefficients_y);
  nav_msgs::Path l_plan;
  l_plan.header.frame_id = "r1/odom";
  l_plan.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped temp_pose;
  temp_pose.header.frame_id = l_plan.header.frame_id;
  temp_pose.header.stamp = l_plan.header.stamp;
  tf2::Quaternion temp_quat;

  int num_point = static_cast<int>(T*10);
  std::cout << "numpoints : " << num_point << std::endl;
  for (int i = 0; i < num_point; i++){
    double x,y;
    x = outputPolynomial(i*0.1, coefficients_x);
    y = outputPolynomial(i*0.1, coefficients_y);
    temp_pose.pose.position.x = x;
    temp_pose.pose.position.y = y;
    double yaw;
    if(i == num_point-1){
      yaw = atan2(req.goal_vel.linear.y, req.goal_vel.linear.x);
    }else{
      double x_1,y_1;
      x_1 = outputPolynomial((i+1)*0.1, coefficients_x);
      y_1 = outputPolynomial((i+1)*0.1, coefficients_y);
      yaw = atan2(y_1-y, x_1-x);
    }
    temp_quat.setRPY(0,0,yaw); 
    temp_pose.pose.orientation.x = temp_quat[0];
    temp_pose.pose.orientation.y = temp_quat[1];
    temp_pose.pose.orientation.z = temp_quat[2];
    temp_pose.pose.orientation.w = temp_quat[3];
    l_plan.poses.push_back(temp_pose);
  }
  pub_polynomial_plan_.publish(l_plan);

  return true;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "quintic_polynomials_server");
  ros::NodeHandle nh;
  ros::ServiceServer solve_polynomial_cb_ = nh.advertiseService("get_polynomials", solvePolynomials);
  pub_polynomial_plan_ = nh.advertise<nav_msgs::Path>("local_plan",1);
  ros::spin();

  return 0;
}
