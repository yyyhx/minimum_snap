#include "qp_test/qp_test.h"
#include "ros/ros.h"
#include <chrono>

class Test {
public:
  Test(const std::string &name, const bool &use_uniform_mode)
      : name_(name), use_uniform_mode_(use_uniform_mode) {
    private_nh_ = ros::NodeHandle(name_);
    path_pub_ =
        private_nh_.advertise<nav_msgs::Path>("minimum_snap/path", 1, true);
    origin_path_pub_ = private_nh_.advertise<nav_msgs::Path>(
        "minimum_snap/origin_path", 1, true);
    origin_path_sub_ =
        nh_.subscribe("rrt/optimal_path", 1, &Test::pathCallback, this);
  }

  void pathCallback(const nav_msgs::PathConstPtr &path) {
    std::vector<double> x_vec;
    std::vector<double> y_vec;
    nav_msgs::Path origin_path;
    origin_path.header.frame_id = "map";
    for (int i = 0; i < path->poses.size(); i++) {
      x_vec.push_back(path->poses[i].pose.position.x);
      y_vec.push_back(path->poses[i].pose.position.y);
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = path->poses[i].pose.position.x;
      pose.pose.position.y = path->poses[i].pose.position.y;
      pose.pose.orientation.z = 1;
      origin_path.poses.push_back(pose);
    }

    auto start = std::chrono::high_resolution_clock::now();
    minimum_snap::QpTest qp_solver;
    auto minimum_snap_path =
        qp_solver.SolverMinimumSnap(x_vec, y_vec, use_uniform_mode_);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_us =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count();
    std::cout << "耗时（秒）: " << duration_us << std::endl;
    path_pub_.publish(minimum_snap_path);
    origin_path_pub_.publish(origin_path);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher path_pub_;
  ros::Publisher origin_path_pub_;
  ros::Subscriber origin_path_sub_;
  std::string name_;
  bool use_uniform_mode_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "minimum_snap_node");
  ros::NodeHandle nh;
  ros::Publisher path_pub_ = nh.advertise<nav_msgs::Path>("path", 1, true);
  ros::Publisher origin_path_pub_ =
      nh.advertise<nav_msgs::Path>("origin_path", 1, true);

  std::vector<double> x_vec;
  std::vector<double> y_vec;

  for (double i = 3.14159; i <= 2 * 3.14159; i += 0.1) {
    double rand_acc = (rand() % 3) * 0.1;
    double r = 2 + rand_acc * 5;

    x_vec.push_back(r * cos(i) + rand_acc);
    y_vec.push_back(r * sin(i));
  }

  for (double i = 3.14159; i >= 0; i -= 0.1) {
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
  auto path = qp_solver.SolverMinimumSnap(x_vec, y_vec, false);
  auto end = std::chrono::high_resolution_clock::now();
  auto duration_us =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
          .count();
  std::cout << "耗时（秒）: " << duration_us << std::endl;
  path_pub_.publish(path);
  origin_path_pub_.publish(origin_path);
  //  Test test("use_uniform_mode", true);
  //  Test test1("Trapezoidal_mode", false);
  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  return 0;
}
