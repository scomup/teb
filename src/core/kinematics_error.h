// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2016 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: vitus@google.com (Michael Vitus)

#ifndef TEB_KINEMATICES_ERROR_PLANNER_H_
#define TEB_KINEMATICES_ERROR_PLANNER_H_

#include <cmath>
#include "ceres/ceres.h"

namespace teb_demo
{

class KinematicsError
{
public:
  KinematicsError(const Eigen::Matrix2d &sqrt_information) : sqrt_information_(sqrt_information) {}

  template <typename T>
  bool operator()(const T *const pose_1,
                  const T *const pose_2,
                  T *residuals_ptr) const
  {

    Eigen::Matrix<T, 2, 1> conf1 =
        Eigen::Map<const Eigen::Matrix<T, 2, 1>>(pose_1);

    const Eigen::Matrix<T, 2, 1> conf2 =
        Eigen::Map<const Eigen::Matrix<T, 2, 1>>(pose_2);

    Eigen::Matrix<T, 2, 1> deltaS = conf2 - conf1;

    Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals_map(residuals_ptr);
    residuals_map(0) = std::fabs<T>((cos(T(pose_1[2])) + cos(T(pose_2[2]))) * deltaS(1) -
                                    (sin(T(pose_1[2])) + sin(T(pose_2[2]))) * deltaS(0));

    Eigen::Matrix<T, 2, 1> angle_vec(cos(T(pose_1[2])), sin(T(pose_1[2])));
    T dir = deltaS.dot(angle_vec);
    residuals_map(1) = dir > T(0) ? T(0) : -dir;

    return true;
  }

  static ceres::CostFunction *Create(const Eigen::Matrix2d &sqrt_information)
  {
    return (new ceres::AutoDiffCostFunction<KinematicsError, 2, 3, 3>(new KinematicsError(sqrt_information)));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  const Eigen::Matrix2d sqrt_information_;
};


} // namespace teb_demo

#endif // CERES_EXAMPLES_POSE_GRAPH_2D_NORMALIZE_ANGLE_H_
