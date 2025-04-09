//
// Created by yyy on 8/26/24.
//

#ifndef SRC_QP_TEST_H
#define SRC_QP_TEST_H

// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"
#include "osqp/osqp.h"
// eigen
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "qp_test/ContinuousTimeSeries.h"
#include "tf/tf.h"

#define N_ 5
#define K_ 4
#define T_ 2

namespace minimum_snap {

class QpTest {
public:
  QpTest() {
    max_v_ = 0.6;
    a_ = 0.3;
  };

  ~QpTest(){};

  nav_msgs::Path SolverMinimumSnap(const std::vector<double> &x_vec,
                                   const std::vector<double> &y_vec,
                                   const bool &use_uniform_mode);

  void SolverPro();

  void Solver();

  void SolverMerage();

  Eigen::VectorXd SolverMerage(const std::vector<double> &vec,
                               const std::vector<double> &t_def_vec);

  std::vector<double> GetTimeDef(const std::vector<double> &x_vec,
                                 const std::vector<double> &y_vec,
                                 const bool &use_uniform_mode);
  /**
   * @brief 求阶乘函数
   */
  double NFactorial(const int &n);

  /**
   * @brief 获取n次项的k阶导数的系数 dk(t^N)/dt
   * @param n n次项系数
   * @param k k阶多项式
   * @return
   */
  double GetKCoefficient(const int &n, const int &k);

  /**
   * @brief    N次多项式K阶导数系数的矩阵 --> [a0,a1,a2,a3....an]
   * @param N  N次多项式
   * @param k  k阶导数
   * @param t  当前点在当前段的时间
   * @return
   */
  Eigen::MatrixXd GetDerivativeMatrix(const int &N, const int &k,
                                      const double &t = 1);
  /**
   * @brief  求解矩阵
   * @param coefficient 矩阵系数
   * @param k  k阶导数
   * @param t 时间
   * @return

   */
  double SolverMatrix(Eigen::Matrix<double, Eigen::Dynamic, 1> coefficient,
                      const int &k, const double &t);

  /**
   * @brief  获取Q矩阵
   * @param n N次多项式
   * @param k  k阶导数
   * @param k k阶导数
   * @return
   */
  Eigen::MatrixXd GetQMatrix(const int &n, const int &k);

  /**
   * @brief  获取Q矩阵
   * @param n N次多项式
   * @param k  k阶导数
   * @param k k阶导数
   * @return
   */
  Eigen::MatrixXd GetMerageQMatrix(const int &pose_num, const int &n,
                                   const int &k);

  Eigen::Matrix<double, Eigen::Dynamic, 1>
  GetMerageUMatrix(const std::vector<double> &vec, const double &start_vel,
                   const double &start_acc);

  /**
   * 连续性约束   两条连续的轨迹起点和终点的jerk相同
   * @brief    微分约束 ：N-2 段，每一段的起点终点固定 --> (N-2)
   * @brief    连续性约束(N-2)个中间点每个点需要满足,x , v，a连续 --> 3(N-2)
   * @brief    边界点约束x, v，a已知3*2
   * @brief    (N-2) + 3(N-2) + 4 = 4N -2
   * @param n  交界处n阶多项式k阶连续
   * @param k
   * @param t 每一段的时间
   * @return
   */
  Eigen::MatrixXd GetMerageAMatrix(const int &n, const int &k,
                                   const std::vector<double> &t);

public:
  int constraint_num_;
  double max_v_;
  double a_;
  double T; //时间
};

} // namespace minimum_snap

#endif // SRC_QP_TEST_H
