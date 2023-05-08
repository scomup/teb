#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include "src/common/misc.h"
#include "src/teb/penalties.h"
#include "numerical_derivative.h"


namespace teb_demo
{
  class TimeFactor : public gtsam::NoiseModelFactor1<gtsam::Vector1>
  {
  protected:
    typedef gtsam::NoiseModelFactor1<gtsam::Vector1> Base;

  public:
    virtual ~TimeFactor() {}

    TimeFactor(
        const gtsam::Key &xi_key,
        const gtsam::SharedNoiseModel &noiseModel)
        : Base(noiseModel, xi_key)
    {
    }


    Eigen::Matrix<double,1,1> error(const gtsam::Vector1 &x) const
    {
      Eigen::Matrix<double,1,1> res = x;
      return res;
    }

    gtsam::Vector evaluateError(const gtsam::Vector1 &x,
                         boost::optional<gtsam::Matrix &> H = boost::none) const
    {
      
      gtsam::Vector1 r = error(x);

      if (H)
      {
        auto func = [&](const gtsam::Vector1& x) { return error(x); };
        *H = numericalDerivativeNM<1, 1>(func, x);
      }

      return r;
    };
  };
}// gtsam
