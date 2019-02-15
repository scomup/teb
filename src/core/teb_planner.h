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

#include <cmath>
#include "src/transform/timestamped_transform.h"
#include "ceres/ceres.h"
#include "src/core/types.h"
#include "src/core/kinematics_error.h"
#include "src/core/obstacle_error.h"
#include "src/core/obstacles.h"

namespace teb_demo
{

class TEBPlanner
{
public:
  TEBPlanner(std::vector<Pose2d> path, std::vector<Obst2d> obstacles):paths_(path),obsts_(obstacles)
  {
  }

void slove(){
    ceres::Problem problem;

    ceres::LossFunction *loss_function = NULL;

    std::cout << "\nintput path:\n";
    std::vector<Pose2d> path;
    for (auto &point : paths_)
    {
      path.push_back(Pose2d{point.x,
                            point.y,
                            point.yaw_radians});
    }

    std::vector<Obstacle*> obstacles;
    for (auto &obst : obsts_)
    {
      obstacles.push_back(new PointObstacle(obst.x, obst.x));
    }


    for (auto &point : path)
    {
      std::cout << point.x << "," << point.y << "," << point.yaw_radians << "\n";
    }

    for (size_t i = 0; i < path.size()-1; i++)
    {
      Eigen::Matrix2d information;
      information << 1000, 0, 0, 1;

      auto &current = path[i];
      auto &next = path[i + 1];
      const Eigen::Matrix2d sqrt_information =
          information.llt().matrixL();

      ceres::CostFunction *cost_function = KinematicsError::Create(sqrt_information);

      problem.AddResidualBlock(cost_function, loss_function,
                               reinterpret_cast<double *>(&current),
                               reinterpret_cast<double *>(&next));
    }

    BaseRobotFootprintModel* robot = new PointRobotFootprint;

    for (size_t i = 0; i < obstacles.size(); i++)
    {
      ceres::CostFunction *cost_function = ObstacleError::Create(robot, obstacles[i]);

      for (size_t j = 0; j < path.size(); j++)
      {
        auto &current = path[j];

        problem.AddResidualBlock(cost_function, NULL,
                                 reinterpret_cast<double *>(&current));
      }
    }
    

    for (size_t i = 0; i < path.size(); i++)
    {
      auto &current = path[i];
      if (i == 0 || i == path.size() - 1)
        problem.SetParameterBlockConstant(reinterpret_cast<double *>(&current));
    }

    ceres::Problem::EvaluateOptions evaluate_options;

    double total_cost = 0.0;
    std::vector<double> residuals;

    problem.Evaluate(evaluate_options, &total_cost, &residuals, nullptr, nullptr);
    std::cout << "total_cost:" << total_cost << "\n";


    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << "\noutput path:\n";
   for (auto &point : path)
    {
      std::cout << point.x << "," << point.y << "," << point.yaw_radians << "\n";
    }
    problem.Evaluate(evaluate_options, &total_cost, &residuals, nullptr, nullptr);
    std::cout << "total_cost:" << total_cost << "\n";
    
  }


  private:
  std::vector<Pose2d> paths_; 
  std::vector<Obst2d> obsts_;

};

} // namespace teb_demo

#endif // CERES_EXAMPLES_POSE_GRAPH_2D_NORMALIZE_ANGLE_H_
