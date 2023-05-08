#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "src/common/misc.h"
#include "src/teb/penalties.h"
#include "numerical_derivative.h"

#include "src/teb/obstacles.h"
#include "src/teb/robot_footprint_model.h"


namespace teb_demo
{
  class ObstacleFactor : public gtsam::NoiseModelFactor1<gtsam::Pose2>
  {
  protected:
    typedef gtsam::NoiseModelFactor1<gtsam::Pose2> Base;
    const BaseRobotFootprintModel *robot_model_; //!< Store pointer to robot_model
    const Obstacle *obstacle_;                   //!< Store pointer to robot_model
    const double min_dist_;
    const double epsilon_;

  public:
    virtual ~ObstacleFactor() {}

    ObstacleFactor(
        const gtsam::Key &xi_key,
        const BaseRobotFootprintModel *robot_model,
        const Obstacle *obstacle,
        const double min_dist,
        const double epsilon,
        const gtsam::SharedNoiseModel &noiseModel)
        : Base(noiseModel, xi_key),
          robot_model_(robot_model),
          obstacle_(obstacle),
          min_dist_(min_dist),
          epsilon_(epsilon)
    {
    }


    Eigen::Matrix<double,1,1> error(const gtsam::Pose2 &x) const
    {
      Eigen::Matrix<double,1,1> res;
      PoseSE2 pose(x.x(), x.y(), x.theta());
      double dist = robot_model_->calculateDistance(pose, obstacle_);
      res(0) = penaltyBoundFromBelow<double>(dist, min_dist_, epsilon_);
      return res;
    }

    gtsam::Vector evaluateError(const gtsam::Pose2 &x,
                         boost::optional<gtsam::Matrix &> H = boost::none) const
    {
      
      gtsam::Vector1 r = error(x);

      if (H)
      {
        auto func = [&](const gtsam::Pose2& x) { return error(x); };
        *H = numericalDerivative<1>(func, x);
      }

      return r;
    };
  };
}// gtsam
