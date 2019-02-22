#ifndef EDGE_OBSTACLE_H_
#define EDGE_OBSTACLE_H_

#include "src/teb/obstacles.h"
#include "src/teb/robot_footprint_model.h"
#include "src/teb/penalties.h"

namespace teb_demo
{

class ObstacleError
{
public:
  ObstacleError(const Eigen::Matrix<double, 1, 1> &information,
                const BaseRobotFootprintModel *robot_model,
                const Obstacle *obstacle,
                const double min_dist,
                const double epsilon)
      : sqrt_information_(information.llt().matrixL()),
        robot_model_(robot_model),
        obstacle_(obstacle),
        min_dist_(min_dist),
        epsilon_(epsilon) {}

  bool operator()(const double *const x,
                  const double *const y,
                  const double *const yaw,
                  double *residuals_ptr) const
  {

    PoseSE2 pose(*x, *y, *yaw);

    double dist = robot_model_->calculateDistance(pose, obstacle_);

    Eigen::Map<Eigen::Matrix<double, 1, 1>> residuals_map(residuals_ptr);
    residuals_map(0) = penaltyBoundFromBelow<double>(dist, min_dist_, epsilon_);

    //std::cout<<residuals_map(0)<<std::endl;
    residuals_map = sqrt_information_ * residuals_map;

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Matrix<double, 1, 1> &information,
                                     const BaseRobotFootprintModel *robot_model,
                                     const Obstacle *obstacle,
                                     const double min_dist,
                                     const double epsilon)
  {
    return (new ceres::NumericDiffCostFunction<ObstacleError, ceres::FORWARD, 1, 1, 1, 1>(
        new ObstacleError(information, robot_model, obstacle, min_dist, epsilon)));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  const Eigen::Matrix<double, 1, 1> sqrt_information_;
  const BaseRobotFootprintModel *robot_model_; //!< Store pointer to robot_model
  const Obstacle *obstacle_;                   //!< Store pointer to robot_model
  const double min_dist_;
  const double epsilon_;
};

} // namespace teb_demo

#endif
