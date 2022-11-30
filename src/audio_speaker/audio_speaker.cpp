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

#include "audio_speaker/audio_speaker.h"

#include "utils/utils.h"

namespace hobot {
namespace audio {

HBAudioSpeaker::HBAudioSpeaker(const std::string config_path)
    : config_path_(config_path) {}

HBAudioSpeaker::~HBAudioSpeaker() { DeInit(); }

int HBAudioSpeaker::Init() {
  if (is_init_) return 0;
  std::string file = config_path_ + "/speaker_config.json";
  // ParseConfig(file);
  if (speaker_enable_ != 1) {
    RCLCPP_WARN(rclcpp::get_logger("audio_speaker"),
                "speaker disable, do not capture audio!!!");
    return 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("audio_speaker"), "init to capture audio");
  int ret = 0;
  speaker_device_ = alsa_device_allocate();
  if (!speaker_device_) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_speaker"),
                 "alloc speaker device error!");
    ret = -ENOMEM;
    return ret;
  }

  /* init speaker device*/
  speaker_device_->name = "hw:0,1";
  speaker_device_->format = SND_PCM_FORMAT_S16;
  speaker_device_->direct = SND_PCM_STREAM_PLAYBACK;
  speaker_device_->rate = 16000;
  speaker_device_->channels = 1;
  speaker_device_->buffer_time = 0;  // use default buffer time
  speaker_device_->nperiods = 4;
  speaker_device_->period_size = 512;  // 1 period including 1024 frames

  ret = alsa_device_init(speaker_device_);
  if (ret < 0) {
    if (speaker_device_) free(speaker_device_);
    RCLCPP_ERROR(rclcpp::get_logger("audio_speaker"),
                 "alsa_device_init speaker failed. ret = %d", ret);
  }

  RCLCPP_WARN(rclcpp::get_logger("audio_speaker"), "init success");
  // system("rm ./*.pcm -rf");
  if (save_audio) {
    audio_file_.open("./audio_speech.pcm",
                     std::ios::app | std::ios::out | std::ios::binary);
  }

  is_init_ = true;
  return 0;
}

int HBAudioSpeaker::DeInit() {
  RCLCPP_INFO(rclcpp::get_logger("audio_speaker"), "deinit");
  if (!is_init_) return 0;
  if (speaker_device_) {
    alsa_device_deinit(speaker_device_);
    alsa_device_free(speaker_device_);
    speaker_device_ = nullptr;
  }
  if (audio_file_.is_open()) {
    audio_file_.close();
  }
  return 0;
}

int HBAudioSpeaker::PlayAudio(char *data, const int len) {
  snd_pcm_sframes_t frames = snd_pcm_bytes_to_frames(speaker_device_->handle, len);
  return alsa_device_write(speaker_device_, data, frames);
}

int HBAudioSpeaker::Run() {
  if (!is_init_) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_speaker"),
                 "HBAudioSpeaker not init.");
    return -1;
  }

  auto speaker_thread_ =
      std::make_shared<std::thread>(&HBAudioSpeaker::SpeakerWriteThread, this);
  return 0;
}

int HBAudioSpeaker::SpeakerWriteThread() {
  RCLCPP_WARN(rclcpp::get_logger("audio_speaker"), "start to capture audio");
  if (!speaker_device_) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_speaker"),
                 "uac device or speaker device is null");
    return -1;
  }

  int ret = -1;
  snd_pcm_sframes_t frames;
  frames = speaker_device_->period_size;
  int size = snd_pcm_frames_to_bytes(speaker_device_->handle, frames);
  char *buffer = new char[size];
  while (rclcpp::ok()) {
    /* write data to speaker device */
    ret = alsa_device_write(speaker_device_, buffer, frames);
    if (save_audio && audio_file_.is_open()) {
      audio_file_.write(buffer, size);
    }
  }
  RCLCPP_WARN(rclcpp::get_logger("audio_speaker"), "stop capture audio");
  delete[] buffer;
  return 0;
}

int HBAudioSpeaker::ParseConfig(std::string config_file) {
  if (config_file.empty()) return -1;
  RCLCPP_INFO(rclcpp::get_logger("audio_speaker"),
              "hobot speaker config file:%s", config_file.c_str());
  std::ifstream ifs(config_file);
  if (!ifs.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("audio_speaker"),
                 "open config file:%s fail", config_file.c_str());
    return -1;
  }

  auto parse_line = [](std::string value, int &result) {
    size_t nEndPos = value.find(":");
    if (nEndPos == std::string::npos) return;
    value = value.substr(nEndPos + 1);
    result = atoi(value.c_str());
  };

  std::string line;
  while (std::getline(ifs, line)) {
    if (line.find("\"speaker_enable\"") != std::string::npos) {
      parse_line(line, speaker_enable_);
      RCLCPP_WARN(rclcpp::get_logger("audio_speaker"), "speaker enable: %d",
                  speaker_enable_);
    }
  }
  ifs.close();
  return 0;
}

}  // namespace audio
}  // namespace hobot
