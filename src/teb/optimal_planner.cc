#include <cmath>
#include "src/teb/error/kinematics_car_like_factor.h"
#include "src/teb/error/obstacle_factor.h"
#include "src/teb/error/time_factor.h"
#include "src/teb/error/velocity_factor.h"

#include "src/teb/error/kinematics_diff_drvie_error.h"
#include "src/teb/error/kinematics_car_like_error.h"
#include "src/teb/error/obstacle_error.h"
#include "src/teb/error/velocity_error.h"
#include "src/teb/error/time_error.h"




#include "src/teb/obstacles.h"
#include <iostream>
#include <fstream>

#include "src/teb/optimal_planner.h"

namespace teb_demo
{

  using gtsam::symbol_shorthand::T;
  using gtsam::symbol_shorthand::P;

  OptimalPlanner::OptimalPlanner(YAML::Node *config)
      : config_(config)
  {
    autosize_ = (*config_)["autosize"].as<bool>();
    dt_ref_ = (*config_)["dt_ref"].as<double>();
    dt_hysteresis_ = (*config_)["dt_hysteresis"].as<double>();
    min_samples_ = (*config_)["min_samples"].as<int>();
    max_samples_ = (*config_)["max_samples"].as<int>();

    max_vel_x_ = (*config_)["max_vel_x"].as<double>();
    max_vel_x_backwards_ = (*config_)["max_vel_x_backwards"].as<double>();
    max_vel_theta_ = (*config_)["max_vel_theta"].as<double>();
    acc_lim_x_ = (*config_)["acc_lim_x"].as<double>();
    acc_lim_theta_ = (*config_)["acc_lim_theta"].as<double>();
    min_turning_radius_ = (*config_)["min_turning_radius"].as<double>();
    robot_model_ = createRobotFootprint();

    min_obstacle_dist_ = (*config_)["min_obstacle_dist"].as<double>();

    penalty_epsilon_ = (*config_)["penalty_epsilon"].as<double>();
    weight_max_vel_x_ = (*config_)["weight_max_vel_x"].as<double>();
    weight_max_vel_theta_ = (*config_)["weight_max_vel_theta"].as<double>();
    weight_kinematics_nh_ = (*config_)["weight_kinematics_nh"].as<double>();
    weight_kinematics_forward_drive_ = (*config_)["weight_kinematics_forward_drive"].as<double>();
    weight_kinematics_turning_radius_ = (*config_)["weight_kinematics_turning_radius"].as<double>();
    weight_optimaltime_ = (*config_)["weight_optimaltime"].as<double>();
    weight_obstacle_ = (*config_)["weight_obstacle"].as<double>();
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
  double timestep_vel = diff_norm / max_vel_x_; // constant velocity
  double timestep_acc;
  double timestep = 1;

  double max_acc_x = acc_lim_x_;

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


  //todo: liu
  //double yaw = atan2(diff_last[1], diff_last[0]);
  //prev.theta() = yaw;
  time_diffs_.push_back(timestep);
  //values_.insert( T(time_diffs_.size()-1), gtsam::Vector1(timestep));
  //printf("insert T %d\n", time_diffs_.size()-1);

  return true;
}

void OptimalPlanner::addPose(double x, double y, double angle)
{
  poses_.emplace_back(x, y, angle);
  //values_.insert( P(poses_.size()-1), gtsam::Pose2(x, y, angle));
  //printf("insert P %d\n", poses_.size()-1);
  calcTimeDiff();
}

void OptimalPlanner::addObstacle(double x, double y)
{
  obstacles_.push_back(new PointObstacle(x, y));
}

void OptimalPlanner::addKinematicEdges()
{  

  //gtsam::noiseModel::Gaussian::shared_ptr model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << 1, 1, 1).finished());
  gtsam::noiseModel::Gaussian::shared_ptr model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(2) << 1, 1).finished());


  //add kinematics edges
  for (size_t i = 0; i < poses_.size() - 1; i++)
  {

    if (min_turning_radius_ == 0 || weight_kinematics_turning_radius_ == 0)
    {
      //TODO: use diff model
      graph_.emplace_shared<KinematicsCarLikeFactor>(P(i), P(i + 1), min_turning_radius_, model);
    }
    else
    {
      graph_.emplace_shared<KinematicsCarLikeFactor>(P(i), P(i + 1), min_turning_radius_, model);
    }
  }
  int start_id = 0;
  int goal_id = poses_.size() - 1;
  gtsam::noiseModel::Gaussian::shared_ptr fix_noise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << 1e-6, 1e-6, 1e-6).finished());
  graph_.add(gtsam::PriorFactor<gtsam::Pose2>(P(start_id), gtsam::Pose2(poses_[start_id].x(),poses_[start_id].y(),poses_[start_id].theta()), fix_noise));
  graph_.add(gtsam::PriorFactor<gtsam::Pose2>(P(goal_id), gtsam::Pose2(poses_[goal_id].x(),poses_[goal_id].y(),poses_[goal_id].theta()), fix_noise));

}

void OptimalPlanner::addObstacleEdges()
{

  gtsam::noiseModel::Gaussian::shared_ptr model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(2) << 1, 1).finished());


  for (size_t i = 0; i < obstacles_.size(); i++)
  {
    for (size_t j = 0; j < poses_.size(); j++)
    {
      auto &current = poses_[j];
      double dist = obstacles_[i]->getMinimumDistance(current.position());

      if (dist > 4)
        continue;

      graph_.emplace_shared<ObstacleFactor>(P(i),
                                            robot_model_,
                                            obstacles_[i],
                                            min_obstacle_dist_,
                                            model);
    }
  }
}

void OptimalPlanner::addTimeEdges()
{
  gtsam::noiseModel::Gaussian::shared_ptr model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(1) << 1).finished());
  for (size_t i = 0; i < time_diffs_.size(); i++)
  {

    graph_.emplace_shared<TimeFactor>(T(i), model);
  }
}

void OptimalPlanner::addVelocityEdges()
{

  gtsam::noiseModel::Gaussian::shared_ptr model = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(2) << 1, 1).finished());


  for (size_t i = 0; i < poses_.size() - 1; i++)
  {
    graph_.emplace_shared<VelocityFactor>(P(i), P(i + 1), T(i),
                                          max_vel_x_backwards_,
                                          max_vel_x_,
                                          max_vel_theta_,
                                          model);
  }
}

BaseRobotFootprintModel *OptimalPlanner::createRobotFootprint()
{
  const YAML::Node &footprint_model_conifg = (*config_)["footprint_model"];
  std::string model_name = footprint_model_conifg["type"].as<std::string>();

  if (model_name.compare("point") == 0)
  {
    return new PointRobotFootprint();
  }
  // circular
  if (model_name.compare("circular") == 0)
  {
    printf("%s, It is not implemented!\n", model_name.c_str());
    exit(0);
  }
  // line
  if (model_name.compare("line") == 0)
  {
    printf("%s, It is not implemented!\n", model_name.c_str());
    exit(0);
  }
  // two circles
  if (model_name.compare("two_circles") == 0)
  {
    printf("%s, It is not implemented!\n", model_name.c_str());
    exit(0);
  }
  // polygon
  if (model_name.compare("polygon") == 0)
  {
    Point2dContainer polygon;
    const YAML::Node &vertices = footprint_model_conifg["vertices"];
    for (std::size_t i = 0; i < vertices.size(); i++)
    {
      const YAML::Node &point = vertices[i];
      polygon.emplace_back(point[0].as<double>(), point[1].as<double>());
    }
    return new PolygonRobotFootprint(polygon);
  }
  printf("%s, unimplemented!\n", model_name.c_str());
  exit(0);

  return nullptr;
}


void OptimalPlanner::solve()
{

  if (autosize_)
  {
    autoResize(dt_ref_, dt_hysteresis_, min_samples_, max_samples_, false);
  }

  for(int i = 0; i < poses_.size(); i++)
  {
    values_.insert( P(i), gtsam::Pose2(poses_[i].x(), poses_[i].y(), poses_[i].theta()));
  }

  for(int i = 0; i < time_diffs_.size(); i++)
  {
    values_.insert( T(i), gtsam::Vector1(time_diffs_[i]));
  }

  addKinematicEdges();
  addObstacleEdges();
  addVelocityEdges();
  addTimeEdges();

  gtsam::LevenbergMarquardtParams parameters;
  parameters.relativeErrorTol = 1e-5;
  parameters.maxIterations = 100;

  gtsam::LevenbergMarquardtOptimizer optimizer(graph_, values_, parameters);
  gtsam::Values result = optimizer.optimize();

  for (int i = 0; i < poses_.size(); ++i)
  {
    gtsam::Pose2 pose = result.at<gtsam::Pose2>(P(i));
    poses_[i] = PoseSE2(pose.x(), pose.y(), pose.theta());
  }

  for (int i = 0; i < time_diffs_.size(); ++i)
  {
    gtsam::Vector1 dt = result.at<gtsam::Vector1>(T(i));
    time_diffs_[i] = dt(0);
  }


  std::ofstream myfile;
  char src[200];

  myfile.open("/home/liu/workspace/teb/script/new_path.txt");
  for (auto &current : poses_)
  {
    sprintf(src, "PATH: %5.2lf %5.2lf %5.2lf\r\n", current.x(), current.y(), normalize_theta(current.theta()));
    myfile << src;
  }

  for (auto &current : obstacles_)
  {
    auto &pose = current->getCentroid();

    sprintf(src, "OBST: %5.2lf %5.2lf\r\n", pose.x(), pose.y());
    myfile << src;
  }
  myfile.close();


}

} // namespace teb_demo