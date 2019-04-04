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

#include "fusion/probabilistic_fusion/pbf_track_manager.h"

#include <algorithm>

#include "util/log.h"

namespace apollo_perception_standalone {

/*
PbfTrackManager *PbfTrackManager::instance() {
  static PbfTrackManager track_manager;
  return &track_manager;
}
*/

PbfTrackManager::PbfTrackManager() {}

int PbfTrackManager::RemoveLostTracks(bool log_fusion) {
  size_t original_size = tracks_.size();

  for (auto t_iter = tracks_.begin(); t_iter != tracks_.end(); t_iter++) {
    if (!(*t_iter)->IsDead()) {
      continue;
    }
    AWARN_IF(log_fusion) << "\033[1;31mDELETE TRACK\033[0m of ID: "
       << (*t_iter)->GetTrackId(); 
  }
  
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [](PbfTrackPtr p) { return p->IsDead(); }),
                tracks_.end());

  // AWARN_IF(original_size - tracks_.size() != 0 &&log_fusion) 
  //   << "\033[1;31mDELETE TRACK\033[0m of size: " 
  //   << original_size - tracks_.size();

  return original_size - tracks_.size();
}

}  // namespace apollo
