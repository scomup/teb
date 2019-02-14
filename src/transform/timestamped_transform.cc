/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "src/transform/timestamped_transform.h"

namespace teb_demo {
namespace transform {

TimestampedTransform3d Interpolate(const TimestampedTransform3d& start,
                                 const TimestampedTransform3d& end,
                                 const double time) {
  CHECK_LE(start.time, time);
  CHECK_GE(end.time, time);

  const double duration = end.time - start.time;
  const double factor = (time - start.time) / duration;
  const Eigen::Vector3d origin =
      start.transform.translation() +
      (end.transform.translation() - start.transform.translation()) * factor;
  const Eigen::Quaterniond rotation =
      Eigen::Quaterniond(start.transform.rotation())
          .slerp(factor, Eigen::Quaterniond(end.transform.rotation()));
  return TimestampedTransform3d{time, transform::Rigid3d(origin, rotation)};
}

TimestampedTransform2d Interpolate(const TimestampedTransform2d& start,
                                 const TimestampedTransform2d& end,
                                 const double time) {
  CHECK_LE(start.time, time);
  CHECK_GE(end.time, time);

  const double duration = end.time - start.time;
  const double factor = (time - start.time) / duration;
  const Eigen::Vector2d origin =
      start.transform.translation() +
      (end.transform.translation() - start.transform.translation()) * factor;
  Eigen::Rotation2Dd rotation =
      start.transform.rotation().slerp(factor, end.transform.rotation());
  return TimestampedTransform2d{time, transform::Rigid2d(origin, rotation)};
}

}  // namespace transform
}  // namespace cartographer
