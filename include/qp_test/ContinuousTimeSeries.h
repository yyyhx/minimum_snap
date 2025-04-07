#include "ros/ros.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

namespace minimum_snap {
class ContinuousTimeSeries {
public:
  ContinuousTimeSeries(
      const std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> &x_series,
      const std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> &y_series,
      const std::vector<double> &segment_t_series);

  /**
   * @brief 通过二分法寻找到连续时间序列上的t对应的分段轨迹索引
   * @param t
   * @return
   */
  int FindCorrespondingIndex(const double &continuous_t);

  /**
   * @brief 将连续时间序列转化为具体轨迹索引以及在其参照系下的时间segment_t
   * @param continuous_t
   * @return
   */
  std::pair<int, double> getSegmentSeries(const double &continuous_t);

  double getTotalT();

private:
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> x_series_;
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> y_series_;
  std::vector<double> segment_t_series_; // todo 各段轨迹的耗时
  std::vector<double>
      continous_t_series_; // todo 将各段轨迹整合为起点为原点的时间序列
  double total_t_;
};
}