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

#include "audio_speaker/audio_recver_node.h"

#include <fstream>
#include <string>

AudioRecverNode::AudioRecverNode(const std::string& node_name, AudioCbType audio_cb)
    : Node(node_name), audio_cb_(audio_cb) {
  this->declare_parameter<std::string>("audio_msg_sub_topic_name",
                                       audio_msg_sub_topic_name_);

  this->get_parameter<std::string>("audio_msg_sub_topic_name",
                                   audio_msg_sub_topic_name_);
  std::stringstream ss;
  ss << "Parameter:"
     << "\n audio_msg_sub_topic_name: " << audio_msg_sub_topic_name_;
  RCLCPP_WARN(rclcpp::get_logger("AudioRecverNode"), "%s", ss.str().c_str());

  rclcpp::QoS qos(rclcpp::KeepLast(7));
  qos.reliable();
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  audio_subscription_ =
      this->create_subscription<audio_msg::msg::AudioFrame>(
          audio_msg_sub_topic_name_, 10,
          std::bind(&AudioRecverNode::AudioCallback, this,
                    std::placeholders::_1));
  if (save_audio) {
    audio_recv_.open("./audio_recver.pcm",
                     std::ios::app | std::ios::out | std::ios::binary);
  }
}

void AudioRecverNode::AudioCallback(
    const audio_msg::msg::AudioFrame::ConstSharedPtr msg) {
  RCLCPP_WARN(rclcpp::get_logger("AudioRecverNode"), "call audio callback");
  if (msg->frame_type.value != 1) {  // if not audio data, return
    std::cout << "not audio data" << std::endl;
    return;
  }

  if (save_audio && audio_recv_.is_open()) {
    audio_recv_.write((char*)msg->data.data(), msg->data.size());
  }
  if (audio_cb_) {
    audio_cb_(msg);
  }
}
