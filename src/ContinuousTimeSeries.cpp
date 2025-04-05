#include "qp_test/ContinuousTimeSeries.h"

namespace minimum_snap {
ContinuousTimeSeries::ContinuousTimeSeries(
    const std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> &x_series,
    const std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> &y_series,
    const std::vector<double> &segment_t_series)
    : x_series_(x_series), y_series_(y_series),
      segment_t_series_(segment_t_series) {
        double continous_t = 0;
        continous_t_series_.push_back(0);
        for(const auto &t : segment_t_series_){
          continous_t += t;
          continous_t_series_.push_back(continous_t);
        }
};

int ContinuousTimeSeries::FindCorrespondingIndex(const double &continuous_t){
int begin_index = 0;
int end_index = continous_t_series_.size() - 1;

while(true){
  if(begin_index+1==end_index)
    return begin_index;

   int mid_index = (begin_index + end_index) / 2; 
   
   //mid > t --> left
   if(continous_t_series_[mid_index] > continuous_t){
    end_index = mid_index;
   }else{
      begin_index = mid_index;
   }

}
}

std::pair<int,double> ContinuousTimeSeries::getSegmentSeries(const double &continuous_t){
  int index = FindCorrespondingIndex(continuous_t);
  double segment_t =continuous_t - continous_t_series_[index];
  return std::make_pair(index,segment_t);
}
      
}