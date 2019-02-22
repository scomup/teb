#include <cmath>
#include "ceres/ceres.h"
#include "src/teb/error/kinematics_error.h"
#include "src/teb/error/obstacle_error.h"
#include "src/teb/error/velocity_error.h"
#include "src/teb/error/time_error.h"

#include "src/teb/obstacles.h"
#include <iostream>
#include <fstream>

#include "src/teb/optimal_planner.h"

namespace teb_demo
{

OptimalPlanner::OptimalPlanner()
{
  teb_config_ = new OptimalConfig;
  teb_config_->max_vel_x_ = 0.2;
  teb_config_->max_acc_x_ = 1;
  teb_config_->dt_ref_ = 0.3;
  teb_config_->dt_hysteresis_ = 0.1;
  teb_config_->min_samples_ = 3;
  teb_config_->max_samples_ = 500;
}

void OptimalPlanner::autoResize(double dt_ref, double dt_hysteresis, int min_samples, int max_samples, bool fast_mode)
{
  /// iterate through all TEB states and add/remove states!

  bool modified = true;

  for (int rep = 0; rep < 100 && modified; ++rep) // actually it should be while(), but we want to make sure to not get stuck in some oscillation, hence max 100 repitions.
  {
    modified = false;

    for (int i = 0; i < (int)time_diffs_.size(); ++i) // TimeDiff connects Point(i) with Point(i+1)
    {
      if (time_diffs_[i] > dt_ref + dt_hysteresis && (int)time_diffs_.size() < max_samples)
      {

        double newtime = 0.5 * time_diffs_[i];

        time_diffs_[i] = newtime;
        PoseSE2 mid_pose = PoseSE2::average(poses_[i], poses_[i + 1]);
        poses_.insert(poses_.begin() + i + 1, mid_pose);
        time_diffs_.insert(time_diffs_.begin() + i + 1, newtime);
        modified = true;
      }
      else if (time_diffs_[i] < dt_ref - dt_hysteresis && (int)time_diffs_.size() > min_samples) // only remove samples if size is larger than min_samples.
      {
        if (i < ((int)time_diffs_.size() - 1))
        {
          time_diffs_[i + 1] = time_diffs_[i + 1] + time_diffs_[i];
          time_diffs_.erase(time_diffs_.begin() + i);
          poses_.erase(poses_.begin() + i + 1);
        }

        modified = true;
      }
    }
    if (fast_mode)
      break;
  }
}

bool OptimalPlanner::calcTimeDiff()
{

  if (poses_.size() < 2)
    return false;
  auto &curr = poses_.back();
  auto &prev = poses_[poses_.size() - 2];
  Eigen::Vector2d diff_last = curr.position() - prev.position();
  double diff_norm = diff_last.norm();
  double timestep_vel = diff_norm / teb_config_->max_vel_x_; // constant velocity
  double timestep_acc;
  double timestep = 1;

  if (teb_config_->max_acc_x_ != 0)
  {
    timestep_acc = sqrt(2 * diff_norm / (teb_config_->max_acc_x_)); // constant acceleration
    if (timestep_vel < timestep_acc && teb_config_->max_acc_x_)
      timestep = timestep_acc;
    else
      timestep = timestep_vel;
  }
  else
    timestep = timestep_vel;

  if (timestep < 0)
    timestep = 0.2; // TODO: this is an assumption

  double yaw = atan2(diff_last[1], diff_last[0]);
  //if (backwards)
  //  yaw = normalize_theta<double>(yaw + M_PI);
  prev.theta() = yaw;
  time_diffs_.push_back(timestep);
  return true;
}

void OptimalPlanner::addPose(double x, double y, double angle)
{
  poses_.emplace_back(x, y, angle);
  calcTimeDiff();
}

void OptimalPlanner::addObstacle(double x, double y)
{
  obstacles_.push_back(new PointObstacle(x, y));
}

void OptimalPlanner::addKinematicEdges(ceres::Problem &problem)
{
  Eigen::Matrix2d information;
  information << 100, 0, 0, 1;
  //add kinematics edges
  for (size_t i = 0; i < poses_.size() - 1; i++)
  {

    auto &current = poses_[i];
    auto &next = poses_[i + 1];

    ceres::CostFunction *cost_function = KinematicsError::Create(information);

    problem.AddResidualBlock(cost_function, nullptr,
                             &current.x(),
                             &current.y(),
                             &current.theta(),
                             &next.x(),
                             &next.y(),
                             &next.theta());
    //set the start and goal as fixed vectices.
    //set the first node as start
    if (i == 0)
    {
      problem.SetParameterBlockConstant(reinterpret_cast<double *>(&current.x()));
      problem.SetParameterBlockConstant(reinterpret_cast<double *>(&current.y()));
      problem.SetParameterBlockConstant(reinterpret_cast<double *>(&current.theta()));
    }
    //set the last node as goal
    if (i == poses_.size() - 2)
    {
      problem.SetParameterBlockConstant(reinterpret_cast<double *>(&next.x()));
      problem.SetParameterBlockConstant(reinterpret_cast<double *>(&next.y()));
      problem.SetParameterBlockConstant(reinterpret_cast<double *>(&next.theta()));
    }
  }
}

void OptimalPlanner::addTimeEdges(ceres::Problem &problem)
{
  Eigen::Matrix<double, 1, 1> information;
  information << 10;

  for (size_t i = 0; i < time_diffs_.size(); i++)
  {
    auto &dt = time_diffs_[i];

    ceres::CostFunction *cost_function = TimeError::Create(information);

    problem.AddResidualBlock(cost_function, nullptr, &dt);
  }
}

void OptimalPlanner::addVelocityEdges(ceres::Problem &problem)
{
  Eigen::Matrix2d information;
  information << 1000, 0, 0, 100;
  //add kinematics edges
  for (size_t i = 0; i < time_diffs_.size(); i++)
  {

    auto &current = poses_[i];
    auto &next = poses_[i + 1];
    auto &dt = time_diffs_[i];

    ceres::CostFunction *cost_function = VelocityError::Create(information);

    problem.AddResidualBlock(cost_function, nullptr,
                             &current.x(),
                             &current.y(),
                             &current.theta(),
                             &next.x(),
                             &next.y(),
                             &next.theta(),
                             &dt);
  }
}

void OptimalPlanner::addObstacleEdges(ceres::Problem &problem)
{
  Eigen::Matrix<double, 1, 1> information;
  information << 100;

  BaseRobotFootprintModel *robot = new PointRobotFootprint;

  for (size_t i = 0; i < obstacles_.size(); i++)
  {
    ceres::CostFunction *cost_function = ObstacleError::Create(information, robot, obstacles_[i]);

    for (size_t j = 0; j < poses_.size(); j++)
    {
      auto &current = poses_[j];
      double dist = obstacles_[i]->getMinimumDistance(current.position());
      //if (dist > 1)
      //  continue;

      //std::cout << current.x() << "," << current.y() << "," << current.theta() << "\n";
      problem.AddResidualBlock(cost_function, NULL,
                               &current.x(),
                               &current.y(),
                               &current.theta());
    }
  }
}

void OptimalPlanner::solve()
{
  autoResize(teb_config_->dt_ref_, teb_config_->dt_hysteresis_, teb_config_->min_samples_, teb_config_->max_samples_, false);

  ceres::Problem problem;

  addKinematicEdges(problem);
  addObstacleEdges(problem);
  addVelocityEdges(problem);
  addTimeEdges(problem);

  ceres::Problem::EvaluateOptions evaluate_options;
  double total_cost = 0.0;
  std::vector<double> residuals;
  //problem.Evaluate(evaluate_options, &total_cost, &residuals, nullptr, nullptr);
  //std::cout << "before total_cost:" << total_cost << "\n";

  ceres::Solver::Options options;
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  //problem.Evaluate(evaluate_options, &total_cost, &residuals, nullptr, nullptr);
  //std::cout << "after total_cost:" << total_cost << "\n";

  std::ofstream myfile;
  myfile.open("/home/liu/workspace/teb/script/path.txt");
  char src[200];
  for (auto &current : poses_)
  {
    sprintf(src, "PATH: %5.2f %5.2f %5.2f\r\n", current.x(), current.y(), normalize_theta(current.theta()));
    //std::std::cout<< src;
    myfile << src;
  }

  for (auto &current : obstacles_)
  {
    auto &pose = current->getCentroid();

    sprintf(src, "OBST: %5.2f %5.2f %5.2f\r\n", pose.x(), pose.y());
    //std::std::cout<< src;
    myfile << src;
  }

  myfile.close();
}

}