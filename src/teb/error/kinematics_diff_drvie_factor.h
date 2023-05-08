#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "src/common/misc.h"
#include "src/teb/penalties.h"

#include "numerical_derivative.h"


namespace teb_demo
{
  class KinematicsDiffDriveFactor : public gtsam::NoiseModelFactor2<gtsam::Pose2, gtsam::Pose2>
  {
  protected:
    double min_turning_radius_;
    typedef gtsam::NoiseModelFactor2<gtsam::Pose2, gtsam::Pose2> Base;

  public:
    virtual ~KinematicsDiffDriveFactor() {}

    KinematicsDiffDriveFactor(
        const gtsam::Key &xi_key, const gtsam::Key &xj_key,
        const double min_turning_radius,
        const gtsam::SharedNoiseModel &noiseModel)
        : Base(noiseModel, xi_key, xj_key),
          min_turning_radius_(min_turning_radius)
    {
    }

    gtsam::Vector1 error(const gtsam::Pose2 &pose1, const gtsam::Pose2 &pose2) const
    {
      gtsam::Vector1 res;
      const Eigen::Vector2d conf1(pose1.x(), pose1.y());
      const Eigen::Vector2d conf2(pose2.x(), pose2.y());

      const Eigen::Vector2d deltaS = conf2 - conf1;

      const double yaw1 = pose1.theta();
      const double yaw2 = pose2.theta();


      double theta0 = cos(yaw1) * deltaS(1) - sin(yaw1) * deltaS(0);
      double theta1 = -cos(yaw2) * deltaS(1) + sin(yaw2) * deltaS(0);
      res(0) = std::abs(normalize_theta<double>(normalize_theta<double>(theta0) - normalize_theta<double>(theta1)));
        
       return res;
    }

    gtsam::Vector evaluateError(const gtsam::Pose2 &pose1, const gtsam::Pose2 &pose2,
                         boost::optional<gtsam::Matrix &> H1 = boost::none,
                         boost::optional<gtsam::Matrix &> H2 = boost::none) const
    {

      
      gtsam::Vector1 r = error(pose1, pose2);

      if (H1)
      {
        auto func = [&](const gtsam::Pose2& x) { return error(x, pose2); };
        *H1 =  numericalDerivative<1>(func, pose1);;
      }
      if (H2)
      {
        auto func = [&](const gtsam::Pose2& x) { return error(pose1, x); };

        *H2 = numericalDerivative<1>(func, pose2);
      }

      return r;
      
    };
  };
}// gtsam
