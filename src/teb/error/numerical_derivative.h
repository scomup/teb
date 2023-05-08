#pragma once

#include <gtsam/geometry/Pose2.h>

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

template<int output_dim, int input_dim>
Eigen::Matrix<double, output_dim, input_dim> numericalDerivativeNM(std::function< Eigen::Matrix<double, output_dim, 1> (Eigen::Matrix<double, input_dim, 1>)> func, const Eigen::Matrix<double, input_dim, 1>& input) 
{
  Eigen::Matrix<double, output_dim, 1> r = func(input);
  const double delta = 1e-5;
  Eigen::Matrix<double, output_dim, input_dim> J;
    for(int j = 0; j < input_dim; j++)
    {
      Eigen::Matrix<double, input_dim, 1> input_delta = input;
      input_delta(j) += delta;
      func(input_delta);
      J.col(j) = (func(input_delta) - r)/delta;
    }
    return J;
}