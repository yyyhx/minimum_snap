//
// Created by yyy on 8/26/24.
//

#include "qp_test/qp_test.h"
namespace minimum_snap {
double QpTest::NFactorial(const int &n) {
  int result = 1;
  for (int i = n; i > 1; i--) {
    result *= i;
  }
  return result;
}

double QpTest::GetRowCoefficient(const int &i, const int &k, const int &N) {
  // todo increase == 0 continue
  double increase_factor = (i - k + 1);
  if (increase_factor <= 0) {
    return 0;
  }

  // todo 0阶导为1
  if (k == 0)
    return 1;

  double coefficient = GetRowCoefficient(i, k - 1, N) * increase_factor;
  return coefficient;
}

Eigen::MatrixXd QpTest::GetDerivativeMatrix(const int &n, const int &k) {
  Eigen::MatrixXd derivative_matrix(n + 1, 1);
  for (int i = 0; i <= n; i++) {
    derivative_matrix(i, 0) = GetRowCoefficient(i, k, n);
  }
  return derivative_matrix;
}

Eigen::MatrixXd QpTest::GetDerivativeMatrix(const double &t, const int &n,
                                            const int &k) {
  Eigen::MatrixXd derivative_matrix(n + 1, 1);
  for (int i = 0; i <= n; i++) {
    derivative_matrix(i, 0) = GetRowCoefficient(i, k, n);
    int w = i - k;
    if (derivative_matrix(i, 0) == 0 || w <= 0)
      continue;
    derivative_matrix(i, 0) *= pow(t, w);
  }
  return derivative_matrix;
}

Eigen::MatrixXd QpTest::GetQMatrix(const int &n, const int &k) {
  auto derivative_matrix = GetDerivativeMatrix(n, k);
  ROS_INFO_STREAM("[DerivativeMatrix] : " << std::endl << derivative_matrix);
  Eigen::MatrixXd Q_matrix(n + 1, n + 1);

  for (int i = 0; i <= n; i++) {
    for (int j = 0; j <= n; j++) {

      // todo continue zero
      if (derivative_matrix(i, 0) == 0 || derivative_matrix(j, 0) == 0) {
        Q_matrix(i, j) = 0;
        continue;
      }

      Q_matrix(i, j) = derivative_matrix(i, 0) * derivative_matrix(j, 0) /
                       ((i - k) + (j - k) + 1);
    }
  }

  ROS_WARN_STREAM("[Q_Matrix] : " << std::endl << Q_matrix);
  return Q_matrix;
}

Eigen::MatrixXd QpTest::GetMerageQMatrix(const int &pose_num, const int &n,
                                         const int &k) {
  int q_row = (n + 1);                             // q矩阵长度
  int numberOfLineSegments = pose_num - 1;         //线段条数
  int merage_q_row = numberOfLineSegments * q_row; // merage_q矩阵长度

  std::cout << "merage_q_row : " << merage_q_row;
  //所有线段的Q矩阵都一样
  auto Q_matrix = GetQMatrix(N_, K_);
  Eigen::MatrixXd merageQMatrix(merage_q_row, merage_q_row);
  for (int i = 0; i < numberOfLineSegments; i++) {
    merageQMatrix.block(i * q_row, i * q_row, q_row, q_row) = Q_matrix;
  }
  ROS_INFO_STREAM("[merage Q] : " << std::endl << merageQMatrix);
  return merageQMatrix;
}

Eigen::MatrixXd QpTest::GetFixedPointConstraintMatrix(const int &n,
                                                      const int &k,
                                                      const double &t,
                                                      const int &index) {
  ROS_WARN_STREAM(" T : " << t);
  auto X_T_0_matrix = GetDerivativeMatrix(0, n, k);
  auto X_T_1_matrix = GetDerivativeMatrix(t, n, k);
  if (index == 0) { // todo 中间点微分约束只有两个端点的固定点约束
    Eigen::MatrixXd A_Matrix(2, n + 1);
    A_Matrix << X_T_0_matrix.transpose(), X_T_1_matrix.transpose();
    ROS_WARN_STREAM("[A_Matrix] : " << std::endl << A_Matrix);
    return A_Matrix;
  } else if (index == -1) { // todo 起点需要约束起点速度和加速度
    auto V_T_0_matrix = GetDerivativeMatrix(0, n, k + 1);
    auto A_T_0_matrix = GetDerivativeMatrix(0, n, k + 2);
    Eigen::MatrixXd A_Matrix(4, n + 1);
    A_Matrix << X_T_0_matrix.transpose(), V_T_0_matrix.transpose(),
        A_T_0_matrix.transpose(), X_T_1_matrix.transpose();
    ROS_WARN_STREAM("[A_Matrix] : " << std::endl << A_Matrix);
    return A_Matrix;
  } else { // todo 终点需要约束终点速度和加速度
    auto V_T_1_matrix = GetDerivativeMatrix(t, n, k + 1);
    auto A_T_1_matrix = GetDerivativeMatrix(t, n, k + 2);
    Eigen::MatrixXd A_Matrix(4, n + 1);
    A_Matrix << X_T_0_matrix.transpose(), X_T_1_matrix.transpose(),
        V_T_1_matrix.transpose(), A_T_1_matrix.transpose();
    ROS_WARN_STREAM("[A_Matrix] : " << std::endl << A_Matrix);
    return A_Matrix;
  }
}

Eigen::MatrixXd QpTest::GetMerageAMatrix(const int &n, const int &k,
                                         const double &t, const int &index) {}

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

  ROS_INFO_STREAM("5! : " << NFactorial(5));
  ROS_INFO_STREAM("6! : " << NFactorial(6));
  ROS_INFO_STREAM("7! : " << NFactorial(7));
  geometry_msgs::Point p1, p2;
  p1.x = 2;
  p1.y = 1;
  p2.x = 4;
  p2.y = 10.0;
  // todo 目标函数
  auto Q_matrix = GetQMatrix(N_, K_);
  Eigen::Matrix<double, N_ + 1, 1> g;

  //  // todo 约束矩阵和上下限
  Eigen::Matrix<double, 4, N_ + 1> A_matrix;
  A_matrix << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 2, 4, 8, 16, 32, 0, 1, 4,
      12, 32, 80;

  ROS_INFO_STREAM("A_MATRIX: \n" << A_matrix);

  Eigen::Matrix<double, 4, 1> l, u;
  l << p1.x, 1.0, p2.x, 1.0;
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
  geometry_msgs::Point p1, p2, p3;
  p1.x = 1;
  p1.y = 1;
  p2.x = 6;
  p2.y = 10.0;
  p3.x = 10.0;
  p3.y = 5.0;

  // todo 目标函数
  Eigen::MatrixXd merageQMatrix = GetMerageQMatrix(3, N_, K_);
  Eigen::Matrix<double, 2 * (N_ + 1), 1> merage_g;
  ROS_INFO_STREAM("[merage Q] : " << std::endl << merageQMatrix);

  // todo 微分约束矩阵和上下限  --- 经过指定点
  auto length = abs(p2.x - p1.x) + abs(p3.x - p2.x); /// todo 按照长度分配时间
  auto Diff_A1_matrix = GetFixedPointConstraintMatrix(
      N_, 0, T_ * (abs(p2.x - p1.x) / length), -1);
  auto Diff_A2_matrix =
      GetFixedPointConstraintMatrix(N_, 0, T_ * (abs(p3.x - p2.x) / length), 1);

  Eigen::Matrix<double, 4, 1> diff_l1, diff_l2, diff_u1, diff_u2;
  diff_l1 << p1.x, 0, 0, p2.x;
  diff_u1 = diff_l1;
  diff_l2 << p2.x, p3.x, 0, 0;
  diff_u2 = diff_l2;

  Eigen::Matrix<double, 9, 1> merage_u, merage_l;
  merage_u.block(0, 0, 4, 1) = diff_l1;
  merage_u.block(5, 0, 4, 1) = diff_l2;
  merage_l = merage_u;

  // todo 连续性约束 --- 前一段轨迹在当前点的jerk - 后一段轨迹在当前点的jerk=0
  auto temp_matrix =
      GetDerivativeMatrix(T_ * (abs(p2.x - p1.x) / length), N_, 3);
  auto Continue_A1_matrix = temp_matrix.transpose();
  auto temp1_matrix = GetDerivativeMatrix(0, N_, 3);
  auto Continue_A2_matrix = temp1_matrix.transpose();

  ROS_WARN_STREAM("Continue_A1_matrix : " << Continue_A1_matrix);
  ROS_WARN_STREAM("Continue_A2_matrix : " << -Continue_A2_matrix);

  // todo 构建merage A
  Eigen::MatrixXd Merage_A_matrix(9, 2 * (N_ + 1));
  Merage_A_matrix.block(0, 0, 4, N_ + 1) = Diff_A1_matrix;
  Merage_A_matrix.block(4, 0, 1, N_ + 1) = Continue_A1_matrix;
  Merage_A_matrix.block(4, N_ + 1, 1, N_ + 1) = -Continue_A2_matrix;
  Merage_A_matrix.block(5, N_ + 1, 4, N_ + 1) = Diff_A2_matrix;
  ROS_WARN_STREAM("[Merage_A_matrix] : " << Merage_A_matrix.rows() << " , "
                                         << Merage_A_matrix.cols() << std::endl
                                         << Merage_A_matrix);

  // todo 创建osqp求解器
  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(true);
  solver.data()->setNumberOfConstraints(Merage_A_matrix.rows()); // todo A Row
  solver.data()->setNumberOfVariables(Merage_A_matrix.cols());   // todo A Col

  // todo 设置H矩阵
  Eigen::SparseMatrix<double> H_hessian = merageQMatrix.sparseView();
  solver.data()->setHessianMatrix(H_hessian);
  solver.data()->setGradient(merage_g);

  // todo 约束条件和上下界

  Eigen::SparseMatrix<double> A_sparase = Merage_A_matrix.sparseView();
  solver.data()->setLinearConstraintsMatrix(A_sparase);
  solver.data()->setLowerBound(merage_u);
  solver.data()->setUpperBound(merage_l);

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
}