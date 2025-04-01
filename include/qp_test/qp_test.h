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
#include "ros/ros.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

#define N_ 5
#define K_ 3
#define T_ 2

namespace minimum_snap {

class QpTest {
public:
  QpTest(){};
  ~QpTest(){};
  void SolverPro();

  void Solver();

  void SolverMerage();

  /**
   * @brief 求阶乘函数
   */
  double NFactorial(const int &n);

  /**
   * @brief 获取N次多项式第i行的K阶导数的系数  dk(X^/N)/dt = N!/(N-K)!
   * @param i  行
   * @param k  K阶导数
   * @param N N次多项式
   * @return
   */
  double GetRowCoefficient(const int &i, const int &k, const int &N);

  /**
   * @brief    N次多项式K阶导数系数的矩阵
   * @param N  N次多项式
   * @param k  k阶导数
   * @return
   */
  Eigen::MatrixXd GetDerivativeMatrix(const int &N, const int &k);

  /**
   * @brief    N次多项式K阶导数系数的矩阵
   * @param N  N次多项式
   * @param k  k阶导数
   * @return
   */
  Eigen::MatrixXd GetDerivativeMatrix(const double &t, const int &N,
                                      const int &k);

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

  /**
   * @brief    1. 设置固定点约束 --- 位置 速度 加速度 jerk
   * @param n    N次多项式
   * @param k    k阶导数
   * @param t    规划时间
   * @param index [-1] start , [0] mid , [1] end
   * @return
   */
  Eigen::MatrixXd GetFixedPointConstraintMatrix(const int &n, const int &k,
                                                const double &t,
                                                const int &index = 0);

  /**
   * 连续性约束   两条连续的轨迹起点和终点的jerk相同
   * @param n
   * @param k
   * @param t
   * @param index
   * @return
   */
  Eigen::MatrixXd GetMerageAMatrix(const int &n, const int &k, const double &t,
                                   const int &index = 0);

public:
  double T; //时间
};

} // namespace minimum_snap

#endif // SRC_QP_TEST_H
