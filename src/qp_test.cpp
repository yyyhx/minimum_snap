//
// Created by yyy on 8/26/24.
//

#include "qp_test/qp_test.h"
namespace minimum_snap {

std::vector<double> QpTest::GetTimeDef(const std::vector<double> &x_vec,
                                       const std::vector<double> &y_vec,
                                       const bool &use_uniform_mode) {
  if (use_uniform_mode) {
    std::vector<double> t_vec;
    for (int i = 0; i < x_vec.size() - 1; i++) {
      auto dist = hypot(x_vec[i + 1] - x_vec[i], y_vec[i + 1] - y_vec[i]);
      t_vec.push_back(dist / max_v_);
      ROS_INFO_STREAM("i_t : " << i << " , " << t_vec[i]);
    }
    return t_vec;
  }

  double t_mid = max_v_ / a_;
  double x_min = max_v_ * t_mid;

  std::vector<double> t_vec;
  for (int i = 0; i < x_vec.size() - 1; i++) {
    auto dist = hypot(x_vec[i + 1] - x_vec[i], y_vec[i + 1] - y_vec[i]);

    ////todo 匀加速再匀减速
    if (dist <= x_min) {
      t_vec.push_back(sqrt(dist / a_));
    } else { ////todo 匀加速再匀速再匀减速
      double x_left = dist - x_min;
      t_vec.push_back(2 * t_mid + (x_left / max_v_));
    }
    ROS_INFO_STREAM("i_t : " << i << " , " << t_vec[i]);
  }
  return t_vec;
}

double QpTest::NFactorial(const int &n) {
  // todo 0！==1
  int result = 1;
  if (n == 0) {
    return result;
  }

  if (n < 0)
    return -1;

  for (int i = n; i >= 1; i--) {
    result *= i;
  }
  return result;
}

double QpTest::GetKCoefficient(const int &n, const int &k) {
  // todo X^n次项的k阶导数 = X^(n-k)
  // todo result = N!/(N-K)!

  auto n_factorial = NFactorial(n);
  auto n_decrease_1_factorial = NFactorial(n - k);
  if (n_decrease_1_factorial < 0) {
    return 0;
  }
  return n_factorial / n_decrease_1_factorial;
  ;
}

Eigen::MatrixXd QpTest::GetDerivativeMatrix(const int &n, const int &k,
                                            const double &t) {
  Eigen::MatrixXd derivative_matrix(n + 1, 1);
  derivative_matrix.setZero();
  for (int i = 0; i <= n; i++) {
    auto value = GetKCoefficient(i, k);
    if (value == 0) {
      derivative_matrix(i, 0) = 0;
    } else {
      derivative_matrix(i, 0) = value * pow(t, i - k);
    }
  }
  return derivative_matrix;
}

double
QpTest::SolverMatrix(Eigen::Matrix<double, Eigen::Dynamic, 1> coefficient,
                     const int &k, const double &t) {
  int n = coefficient.size() - 1;

  auto derivative_matrix = GetDerivativeMatrix(n, k, t);
  double valus = 0;
  for (int i = 0; i < coefficient.size(); i++) {
    valus += coefficient(i, 0) * derivative_matrix(i, 0);
  }
  return valus;
};

Eigen::MatrixXd QpTest::GetQMatrix(const int &n, const int &k) {
  auto derivative_matrix = GetDerivativeMatrix(n, k);
  Eigen::MatrixXd Q_matrix = derivative_matrix * derivative_matrix.transpose();

  for (int i = 0; i <= n; i++) {
    for (int j = 0; j <= n; j++) {
      auto value = Q_matrix(i, j);
      if (value == 0)
        continue;

      Q_matrix(i, j) = value / (i + j - 2 * k + 1);
    }
  }
  return Q_matrix;
}

Eigen::MatrixXd QpTest::GetMerageQMatrix(const int &pose_num, const int &n,
                                         const int &k) {
  // todo Q是对称矩阵 merage_Q是对称矩阵too
  int segments_num = pose_num - 1;
  Eigen::MatrixXd Q_matrix = GetQMatrix(n, k);
  Eigen::MatrixXd merageQMatrix(segments_num * Q_matrix.rows(),
                                segments_num * Q_matrix.rows());
  merageQMatrix.setZero();
  for (int i = 0; i < merageQMatrix.rows(); i += Q_matrix.rows()) {
    merageQMatrix.block(i, i, Q_matrix.rows(), Q_matrix.rows()) = Q_matrix;
  }

  return merageQMatrix;
}

Eigen::MatrixXd QpTest::GetMerageAMatrix(const int &n, const int &k,
                                         const std::vector<double> &t) {
  // row  = 6 + 4*(pose_num-2)+ = 4N -2 //todo 约束项个数
  // col = (n+1)*(N-1) //todo 每一段未知数个数 * 段数
  int pose_num = t.size() + 1; // todo 轨迹点数
  int N = t.size();            // todo 轨迹段数
  int rows = 4 * pose_num - 2;
  int cols = (n + 1) * N;
  Eigen::MatrixXd merageAMatrix(rows, cols);
  merageAMatrix.setZero();
  for (int i = 0; i < pose_num; i++) {
    // start
    if (i == 0) {
      // x,v,a fixed
      Eigen::MatrixXd startAMatrix(3, n + 1);
      startAMatrix.setZero();
      startAMatrix.block(0, 0, 1, n + 1) =
          GetDerivativeMatrix(n, 0, 0).transpose(); // todo x
      startAMatrix.block(1, 0, 1, n + 1) =
          GetDerivativeMatrix(n, 1, 0).transpose(); // todo v
      startAMatrix.block(2, 0, 1, n + 1) =
          GetDerivativeMatrix(n, 2, 0).transpose(); // todo a
      merageAMatrix.block(0, 0, 3, n + 1) = startAMatrix;
    } else if (i < (pose_num - 1)) { // mid
      // todo 1.当前点的固定在当前轨迹起点
      // todo 2.当前点x,v,a和前面一段连续(相减等于0)
      Eigen::MatrixXd MidAMatrix(4, 2 * (n + 1));
      MidAMatrix.setZero();
      // todo x
      MidAMatrix.block(0, n + 1, 1, n + 1) =
          GetDerivativeMatrix(n, 0, 0).transpose();

      // todo x1 = Si-1(T) , x2 = Si(0)) --> x1 , -x2
      MidAMatrix.block(1, 0, 1, n + 1) =
          GetDerivativeMatrix(n, 0, t[i - 1]).transpose();
      MidAMatrix.block(1, n + 1, 1, n + 1) =
          -GetDerivativeMatrix(n, 0, 0).transpose();

      // todo v1 = d1(Si-1(T))/dt v2 = d1(Si(0))/dt --> v1 , -v2
      MidAMatrix.block(2, 0, 1, n + 1) =
          GetDerivativeMatrix(n, 1, t[i - 1]).transpose();
      MidAMatrix.block(2, n + 1, 1, n + 1) =
          -GetDerivativeMatrix(n, 1, 0).transpose();

      // todo a1 = d1(Si-1(T))/dt a2 = d1(Si(0))/dt --> a1 , -a2
      MidAMatrix.block(3, 0, 1, n + 1) =
          GetDerivativeMatrix(n, 2, t[i - 1]).transpose();
      MidAMatrix.block(3, n + 1, 1, n + 1) =
          -GetDerivativeMatrix(n, 2, 0).transpose();

      merageAMatrix.block(3 + 4 * (i - 1), (i - 1) * (n + 1), 4, 2 * (n + 1)) =
          MidAMatrix;
    } else { // end
      // x,v,a fixed
      Eigen::MatrixXd EndAMatrix(3, n + 1);
      EndAMatrix.setZero();
      EndAMatrix.block(0, 0, 1, n + 1) =
          GetDerivativeMatrix(n, 0, t.back()).transpose(); // todo x
      EndAMatrix.block(1, 0, 1, n + 1) =
          GetDerivativeMatrix(n, 1, t.back()).transpose(); // todo v
      EndAMatrix.block(2, 0, 1, n + 1) =
          GetDerivativeMatrix(n, 2, t.back()).transpose(); // todo a

      merageAMatrix.block(rows - 3, cols - (n + 1), 3, n + 1) = EndAMatrix;
    }
  }
  return merageAMatrix;
}

Eigen::Matrix<double, Eigen::Dynamic, 1>
QpTest::GetMerageUMatrix(const std::vector<double> &vec,
                         const double &start_vel, const double &start_acc) {
  int pose_num = vec.size();
  Eigen::Matrix<double, Eigen::Dynamic, 1> merage_u(constraint_num_);
  merage_u.setZero();
  for (int i = 0; i < pose_num; i++) {
    if (i == 0) {
      // todo start x,v,a fixed
      Eigen::Matrix<double, 3, 1> start_u = {vec[0], start_vel, start_acc};
      merage_u.block(0, 0, 3, 1) = start_u;
    } else if (i < (pose_num - 1)) {
      // todo mid Si(0) --fixed
      // todo Si(0)) == Si-1(T) -- v1 -v2 == 0
      // todo d1(Si(0))/dt == d1(Si-1(T))/dt -- v1 -v2 == 0
      // todo d2(Si(0))/dt == d2(Si-1(T))/dt -- a1 - a2 == 0
      Eigen::Matrix<double, 4, 1> mid_u = {vec[i], 0, 0, 0};
      merage_u.block(3 + 4 * (i - 1), 0, 4, 1) = mid_u;
    } else {
      // todo start x,v,a fixed
      Eigen::Matrix<double, 3, 1> end_u = {vec.back(), 0, 0};
      merage_u.block(constraint_num_ - 3, 0, 3, 1) = end_u;
    }
  }
  return merage_u;
}

void QpTest::SolverPro() {
  // todo 目标函数
  Eigen::Matrix<double, 2, 2> H;
  H << 1, -1, -1, 2;
  Eigen::Matrix<double, 2, 1> g;
  g << -2, -6;

  //  // todo 约束矩阵和上下限
  Eigen::Matrix<double, 1, 2> A;
  A << 1, 1;

  Eigen::Matrix<double, 1, 1> l;
  Eigen::Matrix<double, 1, 1> u;
  l << 0;
  u = l;

  // todo 创建osqp求解器
  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(true);
  solver.data()->setNumberOfConstraints(1);
  solver.data()->setNumberOfVariables(2);

  // todo 设置H矩阵
  Eigen::SparseMatrix<double> H_hessian = H.sparseView();
  solver.data()->setHessianMatrix(H_hessian);
  solver.data()->setGradient(g);

  // todo 约束条件和上下界

  Eigen::SparseMatrix<double> A_sparase = A.sparseView();
  solver.data()->setLinearConstraintsMatrix(A_sparase);
  solver.data()->setLowerBound(l);
  solver.data()->setUpperBound(u);

  if (!solver.initSolver()) {
    ROS_ERROR_STREAM("[Solver] : init failed");
    return;
  }

  if (!solver.solve()) {
    ROS_ERROR_STREAM("[Solver] : solver failed");
    return;
  }
  auto solution = solver.getSolution();
  ROS_INFO_STREAM("[solver0] : " << solution(0)
                                 << " , [solver1] : " << solution(1));
}

void QpTest::Solver() {

  geometry_msgs::Point p1, p2;
  p1.x = 2;
  p1.y = 1;
  p2.x = 4;
  p2.y = 10.0;
  // todo 目标函数
  auto Q_matrix = GetQMatrix(N_, K_);
  Eigen::Matrix<double, Eigen::Dynamic, 1> g((N_ + 1));
  g.setZero();

  ROS_INFO_STREAM("g : " << g);
  //  // todo 约束矩阵和上下限
  Eigen::Matrix<double, 4, N_ + 1> A_matrix;
  A_matrix << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 2, 4, 8, 16, 32, 0, 1, 4,
      12, 32, 80;

  ROS_INFO_STREAM("A_MATRIX: \n" << A_matrix);

  Eigen::Matrix<double, 4, 1> l, u;
  l << p1.x, 2.0, p2.x, 1.0;
  u = l;

  // todo 创建osqp求解器
  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(true);
  solver.data()->setNumberOfConstraints(4);    // todo A Row
  solver.data()->setNumberOfVariables(N_ + 1); // todo A Col

  // todo 设置H矩阵
  Eigen::SparseMatrix<double> H_hessian = Q_matrix.sparseView();
  solver.data()->setHessianMatrix(H_hessian);
  solver.data()->setGradient(g);

  // todo 约束条件和上下界

  Eigen::SparseMatrix<double> A_sparase = A_matrix.sparseView();
  solver.data()->setLinearConstraintsMatrix(A_sparase);
  solver.data()->setLowerBound(l);
  solver.data()->setUpperBound(u);

  if (!solver.initSolver()) {
    ROS_ERROR_STREAM("[Solver] : init failed");
    return;
  }

  if (!solver.solve()) {
    ROS_ERROR_STREAM("[Solver] : solver failed");
    return;
  }

  // 检查并输出结果
  if (solver.getSolution().hasNaN()) {
    std::cerr << "Solution contains NaN or infinite values!" << std::endl;
    return;
  }

  auto solution = solver.getSolution();
  ROS_INFO_STREAM("[solver] : " << std::endl << solution);
}

void QpTest::SolverMerage() {
  std::vector<double> x_vec = {1, 2, 0.5, 1.0};
  std::vector<double> y_vec = {1, 2, 3};

  auto t_def_vec = GetTimeDef(x_vec, y_vec, false);
  constraint_num_ = 4 * x_vec.size() - 2;
  // todo 目标函数
  Eigen::MatrixXd merageQMatrix = GetMerageQMatrix(x_vec.size(), N_, K_);

  Eigen::Matrix<double, Eigen::Dynamic, 1> merage_g(t_def_vec.size() *
                                                    (N_ + 1));
  merage_g.setZero();
  // ROS_INFO_STREAM("[merage Q] : " << std::endl << merageQMatrix);
  // todo 微分约束矩阵和上下限  --- 经过指定点
  auto merageAMartrix = GetMerageAMatrix(N_, 3, t_def_vec);
  // ROS_INFO_STREAM("[merage_A] : \n" << merageAMartrix);

  // todo 创建osqp求解器
  //  ROS_WARN_STREAM("NumberOfConstraints : " << merageAMartrix.rows()
  //                                           << " , NumberOfVariables : "
  //                                           << merageAMartrix.cols());
  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(false);
  solver.data()->setNumberOfConstraints(merageAMartrix.rows()); // todo A Row
  solver.data()->setNumberOfVariables(merageAMartrix.cols());   // todo A Col

  // todo 设置H矩阵
  Eigen::SparseMatrix<double> H_hessian = merageQMatrix.sparseView();
  solver.data()->setHessianMatrix(H_hessian);
  solver.data()->setGradient(merage_g);

  // todo 约束条件和上下界
  Eigen::SparseMatrix<double> A_sparase = merageAMartrix.sparseView();
  auto merage_u = GetMerageUMatrix(x_vec, 0.0, 0);
  auto merage_l = merage_u;
  //  ROS_INFO_STREAM("merage_u : \n" << merage_u);
  solver.data()->setLinearConstraintsMatrix(A_sparase);
  solver.data()->setLowerBound(merage_u);
  solver.data()->setUpperBound(merage_l);

  if (!solver.initSolver()) {
    //    ROS_ERROR_STREAM("[Solver] : init failed");
    return;
  }

  if (!solver.solve()) {
    ROS_ERROR_STREAM("[Solver] : solver failed");
    return;
  }

  // 检查并输出结果
  if (solver.getSolution().hasNaN()) {
    std::cerr << "Solution contains NaN or infinite values!" << std::endl;
    return;
  }

  auto solution = solver.getSolution();
  ROS_INFO_STREAM("[solver] : " << std::endl << solution);
}

Eigen::VectorXd QpTest::SolverMerage(const std::vector<double> &vec,
                                     const std::vector<double> &t_def_vec) {

  constraint_num_ = 4 * vec.size() - 2;
  // todo 目标函数
  Eigen::MatrixXd merageQMatrix = GetMerageQMatrix(vec.size(), N_, K_);

  Eigen::Matrix<double, Eigen::Dynamic, 1> merage_g(t_def_vec.size() *
                                                    (N_ + 1));
  merage_g.setZero();
  // todo 微分约束矩阵和上下限  --- 经过指定点
  auto merageAMartrix = GetMerageAMatrix(N_, 3, t_def_vec);
  // ROS_INFO_STREAM("[merage_A] : \n" << merageAMartrix);
  // todo 创建osqp求解器
  OsqpEigen::Solver solver;
  //  solver.settings()->setWarmStart(false);
  solver.settings()->setVerbosity(false);
  solver.data()->setNumberOfConstraints(merageAMartrix.rows()); // todo A Row
  solver.data()->setNumberOfVariables(merageAMartrix.cols());   // todo A Col
  // todo 设置H矩阵
  Eigen::SparseMatrix<double> H_hessian = merageQMatrix.sparseView();
  solver.data()->setHessianMatrix(H_hessian);
  solver.data()->setGradient(merage_g);
  // todo 约束条件和上下界
  Eigen::SparseMatrix<double> A_sparase = merageAMartrix.sparseView();
  auto merage_u = GetMerageUMatrix(vec, 0.0, 0);
  //  auto merage_l = merage_u;
  // ROS_INFO_STREAM("merage_u : \n" << merage_u);
  solver.data()->setLinearConstraintsMatrix(A_sparase);
  solver.data()->setLowerBound(merage_u);
  solver.data()->setUpperBound(merage_u);
  if (!solver.initSolver()) {
    std::cout << "Q = \n" << merageQMatrix;
    ROS_ERROR_STREAM("[Solver] : init failed");
    return Eigen::VectorXd();
  }

  if (!solver.solve()) {
    ROS_ERROR_STREAM("[Solver] : solver failed");
    return Eigen::VectorXd();
  }

  // 检查并输出结果
  if (solver.getSolution().hasNaN()) {
    std::cerr << "Solution contains NaN or infinite values!" << std::endl;
    return Eigen::VectorXd();
  }

  auto solution = solver.getSolution();
  //  solver.clearSolver(); // 可选：重置内部状态
  return solution;
}

nav_msgs::Path QpTest::SolverMinimumSnap(const std::vector<double> &x_vec,
                                         const std::vector<double> &y_vec,
                                         const bool &use_uniform_mode) {
  // todo  x,y解耦分别求解关于t的函数，再去结合同一时刻的求解xy坐标以及航向
  auto t_def_vec = GetTimeDef(x_vec, y_vec, use_uniform_mode);
  auto x_solution = SolverMerage(x_vec, t_def_vec);
  auto y_solution = SolverMerage(y_vec, t_def_vec);

  // todo 将每一段的系数整理
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> x_constraint_vec,
      y_constraint_vec;

  for (int i = 0; i < t_def_vec.size(); i++) {

    x_constraint_vec.emplace_back(x_solution.segment(i * (N_ + 1), N_ + 1));
    y_constraint_vec.emplace_back(y_solution.segment(i * (N_ + 1), N_ + 1));
  }

  ContinuousTimeSeries continuous_time_series(x_constraint_vec,
                                              y_constraint_vec, t_def_vec);

  nav_msgs::Path path;
  path.header.frame_id = "map";
  for (double continuous_t = 0.0;
       continuous_t < continuous_time_series.getTotalT();) {
    auto segment_t_pair = continuous_time_series.getSegmentSeries(continuous_t);
    auto x_coefficient = x_constraint_vec[segment_t_pair.first];
    auto y_coefficient = y_constraint_vec[segment_t_pair.first];

    auto x = SolverMatrix(x_constraint_vec[segment_t_pair.first], 0,
                          segment_t_pair.second);
    auto y = SolverMatrix(y_constraint_vec[segment_t_pair.first], 0,
                          segment_t_pair.second);
    // todo Vx 和 Vy的夹角就是航向
    auto yaw = atan2(SolverMatrix(y_constraint_vec[segment_t_pair.first], 1,
                                  segment_t_pair.second),
                     SolverMatrix(x_constraint_vec[segment_t_pair.first], 1,
                                  segment_t_pair.second));
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    path.poses.push_back(pose);
    continuous_t += 0.05;
  }

  return path;
}

}