#ifndef SPLINE_SMOOTH_FILTER_H
#define SPLINE_SMOOTH_FILTER_H

#include <unsupported/Eigen/Splines>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>

void splineSmooth(const std::vector<geometry_msgs::PoseStamped>& path,
                std::vector<geometry_msgs::PoseStamped>& smooth_plan,
                const int degree){

    const Eigen::Index numKnots = path.size();
    const std::string frame_id = path[0].header.frame_id;
    const ros::Time plan_time = path[0].header.stamp;
    Eigen::RowVectorXd knots(numKnots);
    Eigen::MatrixXd points(2, numKnots);

    for (int i=0; i < numKnots; i++){
        knots[i] = i;
        points.col(i) = Eigen::Vector2d(path[i].pose.position.x, path[i].pose.position.y);
    }
    auto spline = Eigen::SplineFitting<Eigen::Spline<double, 2>>::Interpolate(points, degree, knots);

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = plan_time;
    tf2::Quaternion q;
    q.setRPY(0,0,0);
    pose.pose.orientation = tf2::toMsg(q);
    for (int i = 0; i < numKnots; ++i){
        auto point = spline(knots[i]);
        if (std::isnan(point[0]) || std::isnan(point[1])){
            std::cout << "[SPLINE] data is nan.." << numKnots << std::endl;
        }
        pose.pose.position.x = point[0];
        pose.pose.position.y = point[1];
        if (i < numKnots -1){
            auto next_point = spline(knots[i+1]);
            double dx = next_point[0] - point[0];
            double dy = next_point[1] - point[1];
            double yaw = atan2(dy, dx);
            q.setRPY(0,0,yaw);
            pose.pose.orientation = tf2::toMsg(q);
        }
        smooth_plan.push_back(pose);
    }
}

#endif /* SPLINE_SMOOTH_FILTER_H */
