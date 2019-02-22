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

OptimalPlanner::OptimalPlanner(YAML::Node* config)
: config_(config){}

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
  double max_vel_x = (*config_)["max_vel_x"].as<double>();
  double max_acc_x = (*config_)["max_vel_x"].as<double>();


  if (poses_.size() < 2)
    return false;
  auto &curr = poses_.back();
  auto &prev = poses_[poses_.size() - 2];
  Eigen::Vector2d diff_last = curr.position() - prev.position();
  double diff_norm = diff_last.norm();
  double timestep_vel = diff_norm / max_vel_x; // constant velocity
  double timestep_acc;
  double timestep = 1;

  if (max_acc_x != 0)
  {
    timestep_acc = sqrt(2 * diff_norm / (max_acc_x)); // constant acceleration
    if (timestep_vel < timestep_acc && max_acc_x)
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

  double weight_kinematics_nh = (*config_)["weight_kinematics_nh"].as<double>();
  double weight_kinematics_forward_drive = (*config_)["weight_kinematics_forward_drive"].as<double>();

  Eigen::Matrix2d information;
  information << weight_kinematics_nh, 0, 0, weight_kinematics_forward_drive;
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
  double weight_optimaltime = (*config_)["weight_optimaltime"].as<double>();

  Eigen::Matrix<double, 1, 1> information;
  information << weight_optimaltime;

  for (size_t i = 0; i < time_diffs_.size(); i++)
  {
    auto &dt = time_diffs_[i];

    ceres::CostFunction *cost_function = TimeError::Create(information);

    problem.AddResidualBlock(cost_function, nullptr, &dt);
  }
}

void OptimalPlanner::addVelocityEdges(ceres::Problem &problem)
{
  double weight_max_vel_x = (*config_)["weight_max_vel_x"].as<double>();
  double weight_max_vel_theta = (*config_)["weight_max_vel_theta"].as<double>();

  Eigen::Matrix2d information;
  information << weight_max_vel_x, 0, 0, weight_max_vel_theta;
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

  double weight_obstacle = (*config_)["weight_obstacle"].as<double>();
  double penalty_epsilon = (*config_)["penalty_epsilon"].as<double>();
  double min_obstacle_dist = (*config_)["min_obstacle_dist"].as<double>();

  Eigen::Matrix<double, 1, 1> information;
  information << weight_obstacle;

  BaseRobotFootprintModel *robot = new PointRobotFootprint;

  for (size_t i = 0; i < obstacles_.size(); i++)
  {
    ceres::CostFunction *cost_function =
        ObstacleError::Create(information,
                              robot, obstacles_[i],
                              min_obstacle_dist,
                              penalty_epsilon);

    for (size_t j = 0; j < poses_.size(); j++)
    {
      auto &current = poses_[j];
      double dist = obstacles_[i]->getMinimumDistance(current.position());

      if (dist > 2.5*min_obstacle_dist)
        continue;

      problem.AddResidualBlock(cost_function, NULL,
                               &current.x(),
                               &current.y(),
                               &current.theta());
    }
  }
}

void OptimalPlanner::solve()
{
  char src[200];
  bool autosize = (*config_)["autosize"].as<bool>();

  if (autosize)
  {
    double dt_ref = (*config_)["dt_ref"].as<double>();
    double dt_hysteresis = (*config_)["dt_hysteresis"].as<double>();
    int min_samples = (*config_)["min_samples"].as<int>();
    int max_samples = (*config_)["max_samples"].as<int>();
    autoResize(dt_ref, dt_hysteresis, min_samples, max_samples, false);
  }

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

  for (auto &current : poses_)
  {
    sprintf(src, "PATH: %5.2f %5.2f %5.2f\r\n", current.x(), current.y(), normalize_theta(current.theta()));
    std::cout<< src;
  }
std::cout<< std::endl<< std::endl<< std::endl;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  //problem.Evaluate(evaluate_options, &total_cost, &residuals, nullptr, nullptr);
  //std::cout << "after total_cost:" << total_cost << "\n";

  std::ofstream myfile;
  myfile.open("/home/liu/workspace/teb/script/new_path.txt");
  for (auto &current : poses_)
  {
    sprintf(src, "PATH: %5.2f %5.2f %5.2f\r\n", current.x(), current.y(), normalize_theta(current.theta()));
    std::cout<< src;
    myfile << src;
  }
  

  for (auto &current : obstacles_)
  {
    auto &pose = current->getCentroid();

    sprintf(src, "OBST: %5.2f %5.2f %5.2f\r\n", pose.x(), pose.y());
    std::cout<< src;

    myfile << src;
  }

  myfile.close();
}

}