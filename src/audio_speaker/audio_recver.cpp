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

#include "audio_speaker/audio_recver.h"
#include "audio_engine/audioengine.h"

namespace hobot {
namespace audio {

AudioRecver::AudioRecver(const std::string &node_name,
                               const NodeOptions &options)
    : rclcpp::Node(node_name, options) {
  this->declare_parameter<std::string>("audio_msg_sub_topic_name",
                                       audio_msg_sub_topic_name_);

  this->get_parameter<std::string>("audio_msg_sub_topic_name",
                                   audio_msg_sub_topic_name_);
  std::stringstream ss;
  ss << "Parameter:"
     << "\n audio_msg_sub_topic_name: " << audio_msg_sub_topic_name_;
  RCLCPP_WARN(rclcpp::get_logger("AudioRecver"), "%s", ss.str().c_str());
}

AudioRecver::AudioRecver(const std::string &node_name,
                               const std::string &namespace_,
                               const NodeOptions &options)
    : rclcpp::Node(node_name, namespace_, options) {
 this->declare_parameter<std::string>("audio_msg_sub_topic_name",
                                       audio_msg_sub_topic_name_);

  this->get_parameter<std::string>("audio_msg_sub_topic_name",
                                   audio_msg_sub_topic_name_);
  std::stringstream ss;
  ss << "Parameter:"
     << "\n audio_msg_sub_topic_name: " << audio_msg_sub_topic_name_;
  RCLCPP_WARN(rclcpp::get_logger("AudioRecver"), "%s", ss.str().c_str());
}

AudioRecver::~AudioRecver() {
  RCLCPP_INFO(rclcpp::get_logger("AudioRecver"), "AudioRecver deconstruct");
  DeInit();
}

int AudioRecver::Init() {
  if (is_init_) {
    RCLCPP_ERROR(rclcpp::get_logger("AudioRecver"),
                 "uac server is already init!");
    return 0;
  }

  // create audio speaker node
  audio_speaker_ = std::make_shared<HBAudioSpeaker>();
  int ret = audio_speaker_->Init();
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("AudioRecver"),
                 "speaker init failed, return ret = %d!", ret);
    return ret;
  }

  RCLCPP_WARN(rclcpp::get_logger("AudioRecver"),
              "uac server alsa_device_init success!");

  if (!audio_buffer_) {
    audio_buffer_ = new char[audio_len_];
    memset(audio_buffer_, 0, audio_len_);
  }
  
  rclcpp::QoS qos(rclcpp::KeepLast(7));
  qos.reliable();
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  audio_subscription_ =
      this->create_subscription<audio_msg::msg::AudioFrame>(
          audio_msg_sub_topic_name_, 10,
          std::bind(&AudioRecver::RecvAudio, this,
                    std::placeholders::_1));
  is_init_ = true;
  if (save_audio) {
    std::cout << "to open audio_speak file" << std::endl;
    audio_recv_.open("./audio_recv.pcm",
                     std::ios::app | std::ios::out | std::ios::binary);
    audio_play_.open("./audio_play.pcm",
                     std::ios::app | std::ios::out | std::ios::binary);
  }
  RCLCPP_WARN(rclcpp::get_logger("AudioRecver"), "init success!!!");
  return 0;
}

int AudioRecver::DeInit() {
  if (!is_init_) {
    RCLCPP_ERROR(rclcpp::get_logger("AudioRecver"), "uac server not init!");
    return -1;
  }
    if (play_task_ && play_task_->joinable()) {
    play_task_->join();
    play_task_ = nullptr;
  }
  audio_speaker_->DeInit();
  if (audio_recv_.is_open()) {
    audio_recv_.close();
  }
  if (audio_play_.is_open()) {
    audio_play_.close();
  }
  if (audio_buffer_) delete[] audio_buffer_;
  is_init_ = false;
  return 0;
}

int AudioRecver::Run() {
  if (!is_init_) {
    RCLCPP_ERROR(rclcpp::get_logger("AudioRecver"), "uac server not start!");
    return -1;
  }

  // rclcpp::executors::SingleThreadedExecutor exec;
  // auto play_task = std::make_shared<std::thread>(
  //     std::bind(&AudioRecver::PlayAudio, this));
  // exec.spin();
  // if (play_task && play_task->joinable()) {
  //   play_task.reset();
  // }
  // exec.spin();
  play_task_ = std::make_shared<std::thread>(&AudioRecver::PlayAudio, this);
  return 0;
}


void AudioRecver::RecvAudio(
    const audio_msg::msg::AudioFrame::ConstSharedPtr msg) {
  if (msg->frame_type.value != 1) {  // if not audio data, return
    std::cout << "not audio data" << std::endl;
    return;
  }
  if (!has_recv_) {
    RCLCPP_WARN(rclcpp::get_logger("AudioRecver"),
                "recv first audio frame now!!!");
    has_recv_ = true;
  }
  std::unique_lock<std::mutex> lg(queue_mtx_);
  audio_queue_.emplace(msg);
  if (audio_queue_.size() > queue_len_limit_) {
    RCLCPP_WARN(rclcpp::get_logger("AudioRecver"),
                "smart queue len exceed limit: %d", queue_len_limit_);
    audio_queue_.pop();
  }
  if (save_audio && audio_recv_.is_open()) {
    audio_recv_.write((char*)msg->data.data(), msg->data.size());
  }
  audio_queue_cv_.notify_all();
  lg.unlock();
}

int AudioRecver::PlayAudio() {
  RCLCPP_WARN(rclcpp::get_logger("AudioRecver"),
              "start to write audio to speaker!");
  int ret = 0;
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lg(queue_mtx_);
    audio_queue_cv_.wait(lg);
    if (audio_queue_.empty() || !rclcpp::ok()) {
      continue;
    }
    auto audio_frame = std::move(audio_queue_.front());
    audio_queue_.pop();
    lg.unlock();
 
     if (!AudioEngine::Instance()->IsInit()) {
       std::cout << "sdk not ready" << std::endl;
      // sdk is not ready
      audio_speaker_->PlayAudio((char *)audio_frame->data.data(),
                                audio_frame->data.size());
      if (save_audio && audio_play_.is_open()) {
        audio_play_.write((char *)audio_frame->data.data(),
                                audio_frame->data.size());
      }
      continue;
    }
    
    // audio sdk is running
    ret = AudioEngine::Instance()->ProcessData((char *)audio_frame->data.data(),
                                               audio_frame->data.size(),
                                               audio_buffer_, audio_len_);
    if (ret != 0) {
      continue;
    }

    audio_speaker_->PlayAudio(audio_buffer_, audio_len_);
    if (save_audio && audio_play_.is_open()) {
      audio_play_.write(audio_buffer_, audio_len_);
    }
  }

  return 0;
}

}  // namespace audio
}  // namespace hobot
