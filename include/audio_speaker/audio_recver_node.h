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

#ifndef UAC_SERVER_NODE_H
#define UAC_SERVER_NODE_H

#include <string>
#include "audio_msg/msg/audio_frame.hpp"
#include "rclcpp/rclcpp.hpp"

using AudioCbType = std::function<void(
    const audio_msg::msg::AudioFrame::ConstSharedPtr &msg)>;

class AudioRecverNode : public rclcpp::Node {
 public:
  AudioRecverNode(const std::string &node_name, AudioCbType smart_cb);

 private:
  void AudioCallback(
      const audio_msg::msg::AudioFrame::ConstSharedPtr msg);

  std::string audio_msg_sub_topic_name_ = "/audio";
  AudioCbType audio_cb_ = nullptr;
  rclcpp::Subscription<audio_msg::msg::AudioFrame>::SharedPtr
      audio_subscription_ = nullptr;

  std::ofstream audio_recv_;
  bool save_audio = false;
};

#endif  // AUDIO_CONTROL_NODE_H
