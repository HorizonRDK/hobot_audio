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

#ifndef INCLUDE_AUDIO_RECVER_H_
#define INCLUDE_AUDIO_RECVER_H_

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "utils/alsa_device.h"
#include "audio_msg/msg/audio_frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "audio_speaker/audio_speaker.h"

using rclcpp::NodeOptions;

namespace hobot {
namespace audio {

using rclcpp::NodeOptions;
class AudioRecver : public rclcpp::Node {
 public:
 // node_name为创建的节点名，options为选项，用法和ROS Node相同
  AudioRecver(const std::string& node_name,
                 const NodeOptions& options = NodeOptions());

  AudioRecver(const std::string& node_name, const std::string& namespace_,
                 const NodeOptions& options = NodeOptions());

  virtual ~AudioRecver();

  int Init();
  int DeInit();
  int Run();

 private:
  int PlayAudio();

 private:
  bool is_init_ = false;
  bool has_recv_ = false;

  std::string audio_msg_sub_topic_name_ = "/audio";
  void RecvAudio(
      const audio_msg::msg::AudioFrame::ConstSharedPtr msg);
  rclcpp::Subscription<audio_msg::msg::AudioFrame>::SharedPtr
      audio_subscription_ = nullptr;

  size_t queue_len_limit_ = 20;
  std::queue<audio_msg::msg::AudioFrame::ConstSharedPtr> audio_queue_;
  std::mutex queue_mtx_;
  std::condition_variable audio_queue_cv_;

  std::shared_ptr<HBAudioSpeaker> audio_speaker_ = nullptr;
  char* audio_buffer_ = nullptr;
  int audio_len_ = 16 * 2 * 32;

  bool save_audio = false;
  std::ofstream audio_recv_;
  std::ofstream audio_play_;

  std::shared_ptr<std::thread> play_task_;
};
}  // namespace audio
}  // namespace hobot

#endif
