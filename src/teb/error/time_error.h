#ifndef TEB_PLANNER_TIME_ERROR_H_
#define TEB_PLANNER_TIME_ERROR_H_

#include <cmath>
#include "ceres/ceres.h"
#include "src/teb/penalties.h"
#include "src/common/misc.h"

namespace teb_demo
{

class TimeError
{
public:
  TimeError(const Eigen::Matrix<double, 1, 1> &information)
   : sqrt_information_(information.llt().matrixL()) {}

  template <typename T>
  bool operator()(const T *const deltaT,
                  T *residuals_ptr) const
  {

    Eigen::Map<Eigen::Matrix<T, 1, 1>> residuals_map(residuals_ptr);
  
    residuals_map(0) = *deltaT;

    residuals_map = sqrt_information_.template cast<T>() * residuals_map;

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Matrix<double, 1, 1> &information)
  {
    return (new ceres::AutoDiffCostFunction<TimeError, 1, 1>(new TimeError(information)));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  const Eigen::Matrix<double, 1, 1> sqrt_information_;
};


} // namespace teb_demo

#endif // CERES_EXAMPLES_POSE_GRAPH_2D_NORMALIZE_ANGLE_H_

