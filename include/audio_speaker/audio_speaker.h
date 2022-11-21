// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _INCLUDE_AUDIO_SPEAKER_H_
#define _INCLUDE_AUDIO_SPEAKER_H_

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "untils/alsa_device.h"
#include "rclcpp/rclcpp.hpp"

namespace hobot {
namespace audio {
class HBAudioSpeaker{
 public:
  HBAudioSpeaker(const std::string config_path = "./config");
  virtual ~HBAudioSpeaker();
  int Run();

 public:
  int Init();
  int DeInit();
  int PlayAudio(char* data, const int len);

 private:
  int ParseConfig(std::string config_file);
  int SpeakerWriteThread();

 private:
  bool is_init_ = false;
  std::string config_path_ = "./config";
  int speaker_enable_ = 1;
  std::shared_ptr<std::thread> speaker_thread_;
  alsa_device_t *speaker_device_ = nullptr;
  std::ofstream audio_file_;
  bool save_audio = false;
};

}  // namespace audio
}  // namespace hobot

#endif /* _INCLUDE_UAC_SPEAKER_H_ */
