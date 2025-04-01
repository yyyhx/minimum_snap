#include "qp_test/qp_test.h"
#include "ros/ros.h"
int main(int argc, char **argv) {
  ros::init(argc, argv, "minimum_snap_node");
  ros::NodeHandle nh;
  minimum_snap::QpTest qp_solver;
  qp_solver.Solver();
  // qp_solver.SolverMerage();
  ros::spin();
  return 0;
}
