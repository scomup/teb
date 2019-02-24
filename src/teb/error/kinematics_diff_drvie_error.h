

#ifndef TEB_KINEMATICES_DIFF_DRIVE_ERROR_PLANNER_H_
#define TEB_KINEMATICES_DIFF_DRIVE_ERROR_PLANNER_H_

#include <cmath>
#include "ceres/ceres.h"
#include "src/teb/penalties.h"
#include "Eigen/Core"

namespace teb_demo
{

class KinematicsDiffDriveError
{
public:
  KinematicsDiffDriveError(const Eigen::Matrix2d &information)
      : sqrt_information_(information.llt().matrixL()) {}

  template <typename T>
  bool operator()(const T *const x1, const T *const y1, const T *const yaw1,
                  const T *const x2, const T *const y2, const T *const yaw2,
                  T *residuals_ptr) const
  {

    const Eigen::Matrix<T, 2, 1> conf1(*x1, *y1);

    const Eigen::Matrix<T, 2, 1> conf2(*x2, *y2);

    const Eigen::Matrix<T, 2, 1> deltaS = conf2 - conf1;
    std::cout<<"deltaS"<<":"<<deltaS(0)<<","<<deltaS(1)<<std::endl;
    Eigen::Map<Eigen::Matrix<T, 2, 1> > residuals_map(residuals_ptr);
    residuals_map(0) = ceres::abs((cos(T(*yaw1)) + cos(T(*yaw2))) * deltaS(1) -
                                    (sin(T(*yaw1)) + sin(T(*yaw2))) * deltaS(0));

    Eigen::Matrix<T, 2, 1> angle_vec(cos(T(*yaw1)), sin(T(*yaw1)));
    T dir = deltaS.dot(angle_vec);
    residuals_map(1) = penaltyBoundFromBelow<T>(dir, (T)0, (T)0);
    residuals_map = sqrt_information_.template cast<T>() * residuals_map;

    std::cout<<residuals_map(0)<<","<<residuals_map(1)<<std::endl;

    

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Matrix2d &information)
  {
    return (new ceres::AutoDiffCostFunction<KinematicsDiffDriveError, 2, 1, 1, 1, 1, 1, 1>(new KinematicsDiffDriveError(information)));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  const Eigen::Matrix2d sqrt_information_;
};


} // namespace teb_demo

#endif // CERES_EXAMPLES_POSE_GRAPH_2D_NORMALIZE_ANGLE_H_
