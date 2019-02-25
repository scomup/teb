

#ifndef TEB_KINEMATICES_DIFF_DRIVE_ERROR_PLANNER_H_
#define TEB_KINEMATICES_DIFF_DRIVE_ERROR_PLANNER_H_

#include <cmath>
#include "ceres/ceres.h"
#include "src/teb/penalties.h"
#include "Eigen/Core"
#include "src/common/misc.h"

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
    Eigen::Map<Eigen::Matrix<T, 2, 1> > residuals_map(residuals_ptr);

    T theta0 = cos(T(*yaw1)) * deltaS(1) - sin(T(*yaw1)) * deltaS(0);
    T theta1 = -cos(T(*yaw2)) * deltaS(1) + sin(T(*yaw2)) * deltaS(0);
    residuals_map(0) = ceres::abs(normalize_theta<T>(normalize_theta<T>(theta0) - normalize_theta<T>(theta1)));
    //residuals_map(0) = ceres::abs(normalize_theta<T>(theta0 - theta1));
    /*
    T e1 = ceres::abs(theta0 - theta1);
    T e2 = ceres::abs(normalize_theta<T>(theta0) - normalize_theta<T>(theta1));
    T e3 = ceres::abs(normalize_theta<T>(theta0-theta1));
    T e4 = ceres::abs(normalize_theta<T>(normalize_theta<T>(theta0) - normalize_theta<T>(theta1)));
    if (  ceres::abs(e4 - e1)> 0.000001 )
    {
      std::cout << "-------------------------\n";
      std::cout << "A: x:" << *x1 << " y:" << *y1 << " :theta"
                << ":" << *yaw1 << std::endl;
      std::cout << "B: x:" << *x2 << " y:" << *y2 << " :theta"
                << ":" << *yaw2 << std::endl;
      std::cout << "theta0:" << theta0 << ","
                << "theta1"
                << ":" << theta1 << std::endl;
      std::cout << "residuals_map(0):" << residuals_map(0) << std::endl;
      std::cout << "e1:" << e1 << std::endl;
      std::cout << "e2:" << e2 << std::endl;
      std::cout << "e3:" << e3 << std::endl;
      std::cout << "e4:" << e4 << std::endl;
    }
    */

    Eigen::Matrix<T, 2, 1> angle_vec(cos(T(*yaw1)), sin(T(*yaw1)));
    T dir = deltaS.dot(angle_vec);
    residuals_map(1) = penaltyBoundFromBelow<T>(dir, (T)0, (T)0);
    residuals_map = sqrt_information_.template cast<T>() * residuals_map;

    //std::cout<<residuals_map(0)<<","<<residuals_map(1)<<std::endl;

    

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
