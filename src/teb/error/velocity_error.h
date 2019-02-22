
#ifndef TEB_PLANNER_VELOCITY_ERROR_H_
#define TEB_PLANNER_VELOCITY_ERROR_H_

#include <cmath>
#include "ceres/ceres.h"
#include "src/teb/penalties.h"
#include "src/common/misc.h"

namespace teb_demo
{

class VelocityError
{
public:
  VelocityError(const Eigen::Matrix2d &information)
   : sqrt_information_(information.llt().matrixL()) {}

  template <typename T>
  bool operator()(const T *const x1, const T *const y1, const T *const yaw1,
                  const T *const x2, const T *const y2, const T *const yaw2,
                  const T *const deltaT,
                  T *residuals_ptr) const
  {

    const Eigen::Matrix<T, 2, 1> conf1(*x1, *y1);

    const Eigen::Matrix<T, 2, 1> conf2(*x2, *y2);

    const Eigen::Matrix<T, 2, 1> deltaS = conf2 - conf1;

    const T angle_diff = normalize_theta<T>(*yaw2 - *yaw1);

    T dist = deltaS.norm();
    
    if ( ceres::abs(angle_diff) > (T)0.0001)
    {
      T radius = dist / ((T)2 * sin(angle_diff / (T)2));
      dist = ceres::abs(angle_diff * radius); // actual arg length!
    }
    T vel = dist / *deltaT;
    
    vel *= fast_sigmoid<T>( (T)100 * (deltaS.x()*cos(*yaw1) + deltaS.y()*sin(*yaw1)) ); // consider direction

    Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals_map(residuals_ptr);

    const T omega = angle_diff / *deltaT;
  
    residuals_map(0) = penaltyBoundToInterval<T>(vel, (T)-0.1, (T)0.2 ,(T)0.01);
    residuals_map(1) = penaltyBoundToInterval<T>(omega, (T)0.2,(T)0.01);

    //std::cout<<vel<<","<<residuals_map(0)<<","<<residuals_map(1)<<std::endl;



    residuals_map = sqrt_information_.template cast<T>() * residuals_map;


    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Matrix2d &information)
  {
    return (new ceres::AutoDiffCostFunction<VelocityError, 2, 1, 1, 1, 1, 1, 1, 1>(new VelocityError(information)));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  const Eigen::Matrix2d sqrt_information_;
};


} // namespace teb_demo

#endif // CERES_EXAMPLES_POSE_GRAPH_2D_NORMALIZE_ANGLE_H_
