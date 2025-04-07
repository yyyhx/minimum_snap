#include "qp_test/qp_test.h"
#include "ros/ros.h"
#include <chrono>

int main(int argc, char **argv) {
  ros::init(argc, argv, "minimum_snap_node");
  ros::NodeHandle nh;
  ros::Publisher path_pub_ = nh.advertise<nav_msgs::Path>("path", 1, true);
  ros::Publisher origin_path_pub_ =
      nh.advertise<nav_msgs::Path>("origin_path", 1, true);

  std::vector<double> x_vec;
  std::vector<double> y_vec;

  for (double i = 3.14159; i <= 2 * 3.14159; i += 0.9) {
    double r = 2;
    x_vec.push_back(2 * cos(i));
    y_vec.push_back(2 * sin(i));
  }

  for (double i = 3.14159; i >= 0; i -= 0.9) {
    double r = 2;
    x_vec.push_back(4 + 2 * cos(i));
    y_vec.push_back(0 + 2 * sin(i));
  }

  nav_msgs::Path origin_path;
  origin_path.header.frame_id = "map";
  for (int i = 0; i < x_vec.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x_vec[i];
    pose.pose.position.y = y_vec[i];
    pose.pose.orientation.z = 1;
    origin_path.poses.push_back(pose);
  }

  // qp_solver.Solver();
  //    qp_solver.SolverMerage();
  auto start = std::chrono::high_resolution_clock::now();
  minimum_snap::QpTest qp_solver;
  auto path = qp_solver.SolverMinimumSnap(x_vec, y_vec);
  auto end = std::chrono::high_resolution_clock::now();
  auto duration_us =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();
  std::cout << "耗时（秒）: " << duration_us << std::endl;
  path_pub_.publish(path);
  origin_path_pub_.publish(origin_path);

  ros::spin();
  return 0;
}
