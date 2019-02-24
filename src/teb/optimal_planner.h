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

#ifndef TEB_PLANNER_H_
#define TEB_PLANNER_H_

#include "src/teb/obstacles.h"
#include "src/teb/pose_se2.h"
#include "src/teb/robot_footprint_model.h"
#include "yaml-cpp/yaml.h"

namespace teb_demo
{

class OptimalPlanner
{
public:
  OptimalPlanner(YAML::Node *config);
  void addPose(double x, double y, double angle);
  void addObstacle(double x, double y);
  void solve();

private:
  void autoResize(double dt_ref, double dt_hysteresis, int min_samples, int max_samples, bool fast_mode);
  bool calcTimeDiff();
  void addKinematicEdges(ceres::Problem &problem);
  void addTimeEdges(ceres::Problem &problem);
  void addVelocityEdges(ceres::Problem &problem);
  void addObstacleEdges(ceres::Problem &problem);
  BaseRobotFootprintModel *createRobotFootprint();

  std::vector<PoseSE2> poses_;
  std::vector<Obstacle *> obstacles_;
  std::vector<double> time_diffs_;
  YAML::Node *config_;

  bool autosize_;
  double dt_ref_;
  double dt_hysteresis_;
  double min_samples_;
  double max_samples_;

  double max_vel_x_;
  double max_vel_x_backwards_;
  double max_vel_theta_;
  double acc_lim_x_;
  double acc_lim_theta_;
  double min_turning_radius_;

  BaseRobotFootprintModel *robot_model_;

  double min_obstacle_dist_;

  double penalty_epsilon_;
  double weight_max_vel_x_;
  double weight_max_vel_theta_;
  double weight_kinematics_nh_;
  double weight_kinematics_forward_drive_;
  double weight_kinematics_turning_radius_;
  double weight_optimaltime_;
  double weight_obstacle_;
};

} // namespace teb_demo

#endif // CERES_EXAMPLES_POSE_GRAPH_2D_NORMALIZE_ANGLE_H_
