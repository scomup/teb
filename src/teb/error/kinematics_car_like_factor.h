#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "src/common/misc.h"
#include "src/teb/penalties.h"

#include "numerical_derivative.h"


namespace teb_demo
{
  class KinematicsCarLikeFactor : public gtsam::NoiseModelFactor2<gtsam::Pose2, gtsam::Pose2>
  {
  protected:
    double min_turning_radius_;
    typedef gtsam::NoiseModelFactor2<gtsam::Pose2, gtsam::Pose2> Base;

  public:
    virtual ~KinematicsCarLikeFactor() {}

    KinematicsCarLikeFactor(
        const gtsam::Key &xi_key, const gtsam::Key &xj_key,
        const double min_turning_radius,
        const gtsam::SharedNoiseModel &noiseModel)
        : Base(noiseModel, xi_key, xj_key),
          min_turning_radius_(min_turning_radius)
    {
    }

    Eigen::Vector2d error(const gtsam::Pose2 &pose1, const gtsam::Pose2 &pose2) const
    {
      Eigen::Vector2d res;
      gtsam::Vector2 deltaS = pose2.translation() - pose1.translation();
      double yaw1 = pose1.rotation().theta();
      double yaw2 = pose2.rotation().theta();
      double theta0 = cos(yaw1) * deltaS(1) - sin(yaw1) * deltaS(0);
      double theta1 = -cos(yaw2) * deltaS(1) + sin(yaw2) * deltaS(0);
      res(0) = std::abs(normalize_theta<double>(normalize_theta<double>(theta0) - normalize_theta<double>(theta1)));
      double angle_diff = normalize_theta<double>(yaw2 - yaw1);
      if (std::abs(angle_diff) < 0.0001)
        res(1) = 0; // straight line motion
      else
        res(1) = penaltyBoundFromBelow<double>(std::abs(deltaS.norm() / (2 * sin(angle_diff / 2))), min_turning_radius_, 0.0);
        
       return res;
    }

    gtsam::Vector evaluateError(const gtsam::Pose2 &pose1, const gtsam::Pose2 &pose2,
                         boost::optional<gtsam::Matrix &> H1 = boost::none,
                         boost::optional<gtsam::Matrix &> H2 = boost::none) const
    {

      
      gtsam::Vector2 r = error(pose1, pose2);

      if (H1)
      {
        auto func = [&](const gtsam::Pose2& x) { return error(x, pose2); };
        *H1 =  numericalDerivative<2>(func, pose1);;
      }
      if (H2)
      {
        auto func = [&](const gtsam::Pose2& x) { return error(pose1, x); };

        *H2 = numericalDerivative<2>(func, pose2);
      }

      return r;
      
    };
  };
}// gtsam
