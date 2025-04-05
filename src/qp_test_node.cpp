#include "qp_test/qp_test.h"
#include "ros/ros.h"
int main(int argc, char **argv) {
  ros::init(argc, argv, "minimum_snap_node");
  ros::NodeHandle nh;
  minimum_snap::QpTest qp_solver;
  // qp_solver.Solver();
  // qp_solver.SolverMerage();
  std::vector<double> x_vec = {1, 2, 0.5, 1.0};
  std::vector<double> y_vec = {1, 2, 3, 4};

  qp_solver.SolverMinimumSnap(x_vec, y_vec);
  ros::spin();
  return 0;
}
