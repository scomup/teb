#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "src/common/misc.h"
#include "src/teb/penalties.h"



template<int output_dim>
Eigen::Matrix<double, output_dim, 3> numericalDerivative(std::function< Eigen::Matrix<double, output_dim, 1> (gtsam::Pose2)> func, const gtsam::Pose2& input) 
{
  Eigen::Matrix<double, output_dim, 1> r = func(input);
  const double delta = 1e-5;
  Eigen::Matrix<double, output_dim, 3> J;
    for(int j = 0; j < 3; j++)
    {
      Eigen::Matrix<double, 3, 1> dx;
      dx.setZero();
      dx(j) = delta;
      gtsam::Pose2 input_delta = input * gtsam::Pose2(dx.x(),dx.y(),dx.z()); 
      func(input_delta);
      J.col(j) = (func(input_delta) - r)/delta;
    }
    return J;
}


namespace gtsam
{
  class VelposFactor : public NoiseModelFactor2<Pose2, Pose2>
  {
  protected:
    double min_turning_radius_;
    typedef gtsam::NoiseModelFactor2<Pose2, Pose2> Base;

  public:
    virtual ~VelposFactor() {}

    VelposFactor(
        const Key &xi_key, const Key &xj_key,
        const double min_turning_radius,
        const SharedNoiseModel &noiseModel)
        : Base(noiseModel, xi_key, xj_key),
          min_turning_radius_(min_turning_radius)
    {
    }

    Eigen::Vector2d error(const Pose2 &pose1, const Pose2 &pose2) const
    {
      Eigen::Vector2d res;
      Vector2 deltaS = pose2.translation() - pose1.translation();
      double yaw1 = pose1.rotation().theta();
      double yaw2 = pose2.rotation().theta();

      double theta0 = cos(yaw1) * deltaS(1) - sin(yaw1) * deltaS(0);
      double theta1 = -cos(yaw2) * deltaS(1) + sin(yaw2) * deltaS(0);
      res(0) = std::abs(teb_demo::normalize_theta<double>(teb_demo::normalize_theta<double>(theta0) - teb_demo::normalize_theta<double>(theta1)));
      
      double angle_diff = teb_demo::normalize_theta<double>(yaw2 - yaw1);
      if (std::abs(angle_diff) < 0.0001)
        res(1) = 0; // straight line motion
      else
        res(1) = teb_demo::penaltyBoundFromBelow<double>(ceres::abs(deltaS.norm() / (2 * sin(angle_diff / 2))), min_turning_radius_, 0.0);
        
       return res;
    }

    Vector evaluateError(const Pose2 &pose1, const Pose2 &pose2,
                         boost::optional<Matrix &> H1 = boost::none,
                         boost::optional<Matrix &> H2 = boost::none) const
    {

      
      Vector2 r = error(pose1, pose2);

      if (H1)
      {
        auto func = [&](const Pose2& x) { return error(x, pose2); };
        *H1 = numericalDerivative<2>(func, pose1);
      }
      if (H2)
      {
        auto func = [&](const Pose2& x) { return error(pose1, x); };
        *H1 = numericalDerivative<2>(func, pose2);
      }

      return r;
      
    };
  };
}// gtsam
