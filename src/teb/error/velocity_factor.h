#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "src/common/misc.h"
#include "src/teb/penalties.h"
#include "numerical_derivative.h"


namespace teb_demo
{
  class VelocityFactor : public gtsam::NoiseModelFactor3<gtsam::Pose2,gtsam::Pose2,gtsam::Vector1>
  {
  protected:
    typedef gtsam::NoiseModelFactor3<gtsam::Pose2,gtsam::Pose2,gtsam::Vector1> Base;
    const double max_vel_x_backwards_;
    const double max_vel_x_;
    const double max_vel_theta_;
    const double penalty_epsilon_;

  public:
    virtual ~VelocityFactor() {}

    VelocityFactor(
        const gtsam::Key &xi_key,
        const gtsam::Key &xj_key,
        const gtsam::Key &xk_key,
        const double max_vel_x_backwards,
        const double max_vel_x,
        const double max_vel_theta,
        const gtsam::SharedNoiseModel &noiseModel)
        : Base(noiseModel, xi_key, xj_key, xk_key),
          max_vel_x_backwards_(max_vel_x_backwards),
          max_vel_x_(max_vel_x),
          max_vel_theta_(max_vel_theta),
          penalty_epsilon_(0)

    {
    }

    Eigen::Matrix<double, 2, 1> error(const gtsam::Pose2 &pose1, const gtsam::Pose2 &pose2, const gtsam::Vector1 &dt) const
    {
      
      Eigen::Vector2d res;
      const Eigen::Vector2d conf1(pose1.x(), pose1.y());
      const Eigen::Vector2d conf2(pose2.x(), pose2.y());
      const Eigen::Vector2d deltaS = conf2 - conf1;
      const double yaw1 = pose1.theta();
      const double yaw2 = pose2.theta();

      const double angle_diff = normalize_theta<double>(yaw2 - yaw1);

      double dist = deltaS.norm();

      if (std::abs(angle_diff) > 0.0001)
      {
        double radius = dist / (2 * sin(angle_diff / 2));
        dist = std::abs(angle_diff * radius); // actual arg length!
      }
      double vel = dist / dt(0);

      vel *= fast_sigmoid<double>(100 * (deltaS.x() * cos(yaw1) + deltaS.y() * sin(yaw1))); // consider direction


      const double omega = angle_diff / dt(0);

      res(0) = penaltyBoundToInterval<double>(vel, -max_vel_x_backwards_, max_vel_x_, penalty_epsilon_);
      res(1) = penaltyBoundToInterval<double>(omega, max_vel_theta_, penalty_epsilon_);
      return res;

    }

    gtsam::Vector evaluateError(const gtsam::Pose2 &pose1, const gtsam::Pose2 &pose2, const gtsam::Vector1 &dt,
                         boost::optional<gtsam::Matrix &> H1 = boost::none,
                         boost::optional<gtsam::Matrix &> H2 = boost::none,
                         boost::optional<gtsam::Matrix &> H3 = boost::none) const
    {
      
      gtsam::Vector2 r = error(pose1, pose2, dt);

      if (H1)
      {
        auto func = [&](const gtsam::Pose2 &x) { return error(x, pose2, dt); };
        *H1 = numericalDerivative<2>(func, pose1);
      }

      if (H2)
      {
        auto func = [&](const gtsam::Pose2 &x) { return error(pose1, x, dt); };
        *H2 = numericalDerivative<2>(func, pose2);
      }

      if (H3)
      {
        auto func = [&](const gtsam::Vector1 &x) { return error(pose1, pose2, x); };
        *H3 = numericalDerivativeNM<2, 1>(func, dt);
      }
      
      return r;
    };
  };
}// gtsam
