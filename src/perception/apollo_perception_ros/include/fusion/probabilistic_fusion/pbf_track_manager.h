/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef _APOLLO_PERCEPTION_STANDALONE_FUSION_PROBABILISTIC_FUSION_PBF_TRACK_MANAGER_H_  // NOLINT
#define _APOLLO_PERCEPTION_STANDALONE_FUSION_PROBABILISTIC_FUSION_PBF_TRACK_MANAGER_H_  // NOLINT

#include <vector>

#include "fusion/probabilistic_fusion/pbf_track.h"

namespace apollo_perception_standalone {

class PbfTrackManager {
 public:

  //PbfTrackManager *instance();'

  PbfTrackManager();
  ~PbfTrackManager() = default;

  std::vector<PbfTrackPtr> &GetTracks() { return tracks_; }

  const std::vector<PbfTrackPtr> &GetTracks() const { return tracks_; }

  void AddTrack(PbfTrackPtr track) { tracks_.push_back(track); }

  int RemoveLostTracks(bool log_fusion);

 protected:
  std::vector<PbfTrackPtr> tracks_;

};

}  // namespace apollo

/* clang-format off */
#endif  // MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_TRACK_MANAGER_H_ // NOLINT
/* clang-format on */
