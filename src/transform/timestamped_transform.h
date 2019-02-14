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

#ifndef SAMPLE_CARTO_TRANSFORM_TIMESTAMPED_TRANSFORM_H_
#define SAMPLE_CARTO_TRANSFORM_TIMESTAMPED_TRANSFORM_H_

#include "src/transform/rigid_transform.h"

namespace teb_demo {
namespace transform {

struct TimestampedTransform3d {
  double time;
  transform::Rigid3d transform;
};

struct TimestampedTransform2d {
  double time;
  transform::Rigid2d transform;
};


TimestampedTransform3d Interpolate(const TimestampedTransform3d& start,
                                 const TimestampedTransform3d& end,
                                 const double time);

TimestampedTransform2d Interpolate(const TimestampedTransform2d& start,
const TimestampedTransform2d& end,
const double time);


}  // namespace transform
}  // namespace teb_demo

#endif  // SAMPLE_CARTO_TRANSFORM_TIMESTAMPED_TRANSFORM_H_
