/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 * 
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann
 *********************************************************************/
#ifndef EDGE_OBSTACLE_H_
#define EDGE_OBSTACLE_H_

#include "src/core/obstacles.h"
#include "src/core/robot_footprint_model.h"
//#include "src/core/g2o_types/vertex_pose.h"
//#include "src/core/g2o_types/base_teb_edges.h"
//#include "src/core/g2o_types/penalties.h"
//#include "src/core/teb_config.h"



namespace teb_demo
{

inline double penaltyBoundFromBelow(const double& var, const double& a,const double& epsilon)
{
  if (var >= a+epsilon)
  {
    return 0.;
  }
  else
  {
    return (-var + (a+epsilon));
  }
}

class ObstacleError
{
public:
  ObstacleError(const BaseRobotFootprintModel *robot_model,
                const Obstacle *obstacle) : robot_model_(robot_model),
                                            obstacle_(obstacle) {}

  bool operator()(const double *const pose,
                  double *residuals_ptr) const
  {
    
    PoseSE2 posese2(pose[0], pose[1], pose[2]);

    double dist = robot_model_->calculateDistance(posese2, obstacle_);
    
    residuals_ptr[0] = penaltyBoundFromBelow(dist, 1, 0.001)*10;

    return true;
  }

  static ceres::CostFunction *Create(const BaseRobotFootprintModel *robot_model, const Obstacle *obstacle)
  {
    return (new ceres::NumericDiffCostFunction<ObstacleError, ceres::FORWARD, 1, 3>(new ObstacleError(robot_model, obstacle)));
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  const BaseRobotFootprintModel* robot_model_; //!< Store pointer to robot_model
  const Obstacle* obstacle_; //!< Store pointer to robot_model
  //const Eigen::Matrix2d sqrt_information_;

};

} // end namespace

#endif
