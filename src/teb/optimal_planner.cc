#include <cmath>
#include "src/teb/error/kinematics_diff_drvie_factor.h"
#include "src/teb/error/kinematics_car_like_factor.h"
#include "src/teb/error/obstacle_factor.h"
#include "src/teb/error/time_factor.h"
#include "src/teb/error/velocity_factor.h"
#include "src/teb/obstacles.h"

#include <iostream>
#include <fstream>
#include <boost/format.hpp>

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
    double obstacle_noise = (*config_)["obstacle_noise"].as<double>();
    double time_noise = (*config_)["time_noise"].as<double>();
    double velocity_x_noise = (*config_)["velocity_x_noise"].as<double>();
    double velocity_theta_noise = (*config_)["velocity_theta_noise"].as<double>();
    double kinematic_forward_noise = (*config_)["kinematic_forward_noise"].as<double>();
    double kinematic_turning_noise = (*config_)["kinematic_turning_noise"].as<double>();

    obstacle_noise_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(1) << obstacle_noise).finished());
    time_noise_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(1) << time_noise).finished());
    velocity_noise_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(2) << velocity_x_noise, velocity_theta_noise).finished());
    kinematic_noise1_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(1) << kinematic_forward_noise).finished());
    kinematic_noise2_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(2) << kinematic_forward_noise, kinematic_turning_noise).finished());
    fix_noise_ = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(3) << 1e-6, 1e-6, 1e-6).finished());
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

  time_diffs_.push_back(timestep);

  return true;
}

void OptimalPlanner::addPose(const Eigen::Vector3d& pose)
{
  poses_.emplace_back(pose(0), pose(1), pose(2));
  calcTimeDiff();
}

void OptimalPlanner::addObstacle(const Point2dContainer& obst)
{
  if(obst.size() == 0)
  {
  return;
  }
  else if(obst.size() == 1)
  {
    obstacles_.push_back(new PointObstacle(obst[0]));
  }
  else if(obst.size() == 2)
  {
    obstacles_.push_back(new LineObstacle(obst[0], obst[1]));
  }
  else if(obst.size() >= 3)
  {
    obstacles_.push_back(new PolygonObstacle(obst));
  }


}

void OptimalPlanner::addKinematicEdges()
{  
  //add kinematics edges
  for (size_t i = 0; i < poses_.size() - 1; i++)
  {
    if (min_turning_radius_ == 0)
    {
      //TODO: use diff model
      graph_.emplace_shared<KinematicsDiffDriveFactor>(P(i), P(i + 1), kinematic_noise1_);
      factor_type_.push_back("KinematicsDiffDriveFactor");
    }
    else
    {
      graph_.emplace_shared<KinematicsCarLikeFactor>(P(i), P(i + 1),
                                                     min_turning_radius_,
                                                     kinematic_noise2_);
      factor_type_.push_back("KinematicsCarLikeFactor");
    }
  }
  int start_id = 0;
  int goal_id = poses_.size() - 1;
  graph_.add(gtsam::PriorFactor<gtsam::Pose2>(P(start_id),
                                              gtsam::Pose2(poses_[start_id].x(),
                                                           poses_[start_id].y(),
                                                           poses_[start_id].theta()),
                                              fix_noise_));
  factor_type_.push_back("PriorFactor");
  graph_.add(gtsam::PriorFactor<gtsam::Pose2>(P(goal_id),
                                              gtsam::Pose2(poses_[goal_id].x(),
                                                           poses_[goal_id].y(),
                                                           poses_[goal_id].theta()),
                                              fix_noise_));
  factor_type_.push_back("PriorFactor");
}

void OptimalPlanner::addObstacleEdges()
{
  double dist_fn = std::max(min_obstacle_dist_ * 3, 4.);

  for (size_t i = 0; i < obstacles_.size(); i++)
  {
    for (size_t j = 0; j < poses_.size(); j++)
    {
      double dist = robot_model_->calculateDistance(poses_[j], obstacles_[i]);

      if (dist > dist_fn)
        continue;

      graph_.emplace_shared<ObstacleFactor>(P(j),
                                            robot_model_,
                                            obstacles_[i],
                                            min_obstacle_dist_,
                                            penalty_epsilon_,
                                            obstacle_noise_);
      factor_type_.push_back("ObstacleFactor");
    }
  }
}

void OptimalPlanner::addTimeEdges()
{
  for (size_t i = 0; i < time_diffs_.size(); i++)
  {
    graph_.emplace_shared<TimeFactor>(T(i), time_diffs_[i], time_noise_);
    //graph_.emplace_shared<TimeFactor>(T(i), 0, time_noise_);
    factor_type_.push_back("TimeFactor");
  }
}

void OptimalPlanner::addVelocityEdges()
{

  for (size_t i = 0; i < poses_.size() - 1; i++)
  {
    graph_.emplace_shared<VelocityFactor>(P(i), P(i + 1), T(i),
                                          max_vel_x_backwards_,
                                          max_vel_x_,
                                          max_vel_theta_,
                                          penalty_epsilon_,
                                          velocity_noise_);
    factor_type_.push_back("VelocityFactor");
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

  for(uint32_t i = 0; i < poses_.size(); i++)
  {
    values_.insert( P(i), gtsam::Pose2(poses_[i].x(), poses_[i].y(), poses_[i].theta()));
  }

  for(uint32_t i = 0; i < time_diffs_.size(); i++)
  {
    values_.insert( T(i), gtsam::Vector1(time_diffs_[i]));
  }
  //Constraints of robot kinematics
  addKinematicEdges();
  //Constraints of obstacle distance
  addObstacleEdges();
  //Speed Constraints
  addVelocityEdges();
  //Time Constraints
  addTimeEdges();

  gtsam::LevenbergMarquardtParams parameters;
  parameters.relativeErrorTol = 1e-5;
  parameters.maxIterations = 100;

  gtsam::LevenbergMarquardtOptimizer optimizer(graph_, values_, parameters);
  result_ = optimizer.optimize();


  for (uint32_t i = 0; i < poses_.size(); ++i)
  {
    gtsam::Pose2 pose = result_.at<gtsam::Pose2>(P(i));
    poses_[i] = PoseSE2(pose.x(), pose.y(), pose.theta());
  }

  for (uint32_t i = 0; i < time_diffs_.size(); ++i)
  {
    gtsam::Vector1 dt = result_.at<gtsam::Vector1>(T(i));
    time_diffs_[i] = dt(0);
  }

  std::ofstream myfile;

  myfile.open("../script/new_path.txt");

  for (auto &current : poses_)
  {
    const std::string str = (boost::format("PATH: %5.2f %5.2f %5.2f\n") % current.x() % current.y() % current.theta()).str();
    myfile << str;
  }

  for (auto &current : obstacles_)
  {
    geometry_msgs::Polygon polygon;
    current->toPolygonMsg(polygon);

    std::string str = std::string("OBST: ");

    for(auto point : polygon.points)
    {
      str += (boost::format("%5.2f %5.2f ") % point.x % point.y).str();
    }
    str += std::string("\n");
    myfile << str;
  }
  myfile.close();
}

void OptimalPlanner::report()
{
  double error0 = 0;
  double error1 = 0;
  std::map<std::string, std::vector<double>> type_score;
  for (uint i = 0; i < graph_.size(); i++)
  {
    auto &factor = graph_.at(i);
    double err0 = factor->error(values_);
    double err1 = factor->error(result_);
    if (type_score.count(factor_type_[i]) == 0)
    {
      type_score[factor_type_[i]] = {err0, err1};
    }
    else
    {
      type_score[factor_type_[i]][0] += err0;
      type_score[factor_type_[i]][1] += err1;
    }
    error0 += err0;
    error1 += err1;
  }
  printf("-----------------\n");
  printf("Overall error: %f --> %f.\n", error0, error1);
  for (auto m : type_score)
  {
    auto name = m.first;
    auto score = m.second;
    printf("  --> Factor %s : %f --> %f.\n", name.c_str(), score[0], score[1]);
  }
  printf("-----------------\n");
}

double OptimalPlanner::getCloestDist()
{
  double closest_dist = INFINITY;
  double avg_dist = 0;
  for (size_t i = 0; i < obstacles_.size(); i++)
  {
    for (size_t j = 0; j < poses_.size(); j++)
    {
      double dist = robot_model_->calculateDistance(poses_[j], obstacles_[i]);
      if(closest_dist > dist)
        closest_dist = dist;
      avg_dist += dist;
      
      //printf("the distance form obstacle %d to pose %d is %f\n", i, j, dist);
    }
  }
  printf("The closest distance is %f\n", closest_dist);
  return closest_dist;
}

} // namespace teb_demo