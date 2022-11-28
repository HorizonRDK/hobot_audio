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

#include <memory>
#include <string>

#include "audio_capture/hb_audio_capture.h"
#include "audio_server.h"
#include "audio_speaker/audio_recver.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_WARN(rclcpp::get_logger("audio_capture"),
              "This is audio capture example!");
  std::shared_ptr<hobot::audio::HBAudioCapture> audio_capture = 
     std::make_shared<hobot::audio::HBAudioCapture>("audio_capture");
  if (audio_capture->Init() == 0) {
    if (audio_capture->Run() != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("audio_capture"),
                   "Run HBAudioCapture failed!");
    } else {
      RCLCPP_INFO(rclcpp::get_logger("audio_capture"),
                  "Run HBAudioCapture done!");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("audio_capture"),
                 "Init HBAudioCapture failed!");
  }
  
  std::shared_ptr<hobot::audio::AudioRecver> audio_recver = 
     std::make_shared<hobot::audio::AudioRecver>("audio_recver");
  if (audio_recver->Init() == 0) {
    if (audio_recver->Run() != 0) {
      RCLCPP_ERROR(rclcpp::get_logger("audio_recver"),
                   "Run AudioRecver failed!");
    } else {
      RCLCPP_INFO(rclcpp::get_logger("audio_recver"),
                  "Run AudioRecver done!");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("audio_recver"),
                 "Init AudioRecver failed!");
  }

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(audio_capture);
  exec.add_node(audio_recver);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
