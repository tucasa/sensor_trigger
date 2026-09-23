// Copyright 2022 Tier IV, Inc.
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

#include <sensor_trigger/sensor_trigger.hpp>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <string>

namespace sensor_trigger
{
SensorTrigger::SensorTrigger(const rclcpp::NodeOptions & node_options)
: Node("sensor_trigger", node_options)
{
  int enabled_count = 0;
  const int cpu_count = static_cast<int>(std::thread::hardware_concurrency());
  fps_ = declare_parameter("frame_rate", 10.0);
  pulse_width_ms_ = declare_parameter("pulse_width_ms", 5);
  if (fps_ < 1.0) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Unable to trigger slower than 1 fps. frame_rate is " << fps_ << ".");
    rclcpp::shutdown();
    return;
  }

  for (int index = 0; index < kDeserializerCount; ++index) {
    Channel & channel = channels_[index];
    channel.bus = kMapping[index].bus;
    channel.camera_a = kMapping[index].camera_a;
    channel.camera_b = kMapping[index].camera_b;
    const std::string prefix = "dser" + std::to_string(index) + ".";
    channel.phase = declare_parameter(prefix + "phase", 0.0);
    channel.cpu = declare_parameter(prefix + "cpu_core_id", index + 1);
    channel.enabled = declare_parameter(prefix + "enabled", true);

    if (!channel.enabled) {
      continue;
    }
    ++enabled_count;
  }

  if (enabled_count == 0) {
    RCLCPP_ERROR(get_logger(), "No deserializer is enabled.");
    rclcpp::shutdown();
    return;
  }

  for (int index = 0; index < kDeserializerCount; ++index) {
    Channel & channel = channels_[index];
    if (!channel.enabled) {
      continue;
    }
    if (!channel.output.open(channel.bus)) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Failed to initialize MAX9296 MFP0 on i2c-" << channel.bus << ": "
                                                                  << strerror(errno) << ".");
      rclcpp::shutdown();
      return;
    }
    const std::string topic = "dser" + std::to_string(index) + "/trigger_time";
    channel.trigger_time_publisher =
      create_publisher<builtin_interfaces::msg::Time>(topic, 1000);
    RCLCPP_INFO_STREAM(
      get_logger(), "dser" << index << " triggers cameras " << channel.camera_a << " and "
                           << channel.camera_b << " on i2c-" << channel.bus << " at " << fps_
                           << " Hz, phase " << channel.phase << " deg, cpu " << channel.cpu << ".");
  }

  for (int index = 0; index < kDeserializerCount; ++index) {
    if (!channels_[index].enabled) {
      continue;
    }
    channels_[index].trigger_thread = std::thread(&SensorTrigger::run, this, index);
  }

  sched_param sch;
  int policy;
  for (int index = 0; index < kDeserializerCount; ++index) {
    Channel & channel = channels_[index];
    if (!channel.trigger_thread.joinable()) {
      continue;
    }
    if (channel.cpu < 0 || channel.cpu >= cpu_count) {
      RCLCPP_WARN_STREAM(
        get_logger(), "dser" << index << " CPU core " << channel.cpu << " is not available.");
    } else {
      cpu_set_t cpuset;
      CPU_ZERO(&cpuset);
      CPU_SET(channel.cpu, &cpuset);
      if (pthread_setaffinity_np(
            channel.trigger_thread.native_handle(), sizeof(cpu_set_t), &cpuset)) {
        RCLCPP_WARN_STREAM(
          get_logger(),
          "dser" << index << " failed to set CPU affinity: " << strerror(errno) << ".");
      }
    }
    pthread_getschedparam(channel.trigger_thread.native_handle(), &policy, &sch);
    sch.sched_priority = 30;
    if (pthread_setschedparam(channel.trigger_thread.native_handle(), SCHED_FIFO, &sch)) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Failed to set schedule parameters: " << strerror(errno) << ".");
    }
  }
}

SensorTrigger::~SensorTrigger()
{
  running_.store(false);
  for (auto & channel : channels_) {
    if (channel.trigger_thread.joinable()) {
      channel.trigger_thread.join();
    }
  }
}

void SensorTrigger::run(int index)
{
  Channel & channel = channels_[index];
  builtin_interfaces::msg::Time trigger_time_msg;

  // Start on the first time after TOS
  int64_t start_nsec;
  int64_t end_nsec;
  int64_t target_nsec;
  int64_t interval_nsec = (int64_t)(1e9 / fps_);
  int64_t pulse_width = pulse_width_ms_ * 1e6;  // millisecond -> nanoseconds
  int64_t wait_nsec = 0;
  int64_t now_nsec = 0;
  // Fix this later to remove magic numbers
  if (std::abs(channel.phase) <= 1e-7) {
    start_nsec = 0;
  } else {
    start_nsec = interval_nsec * (int64_t)(channel.phase * 10) / 3600;
  }
  target_nsec = start_nsec;
  end_nsec = start_nsec - interval_nsec + 1e9;

  while (rclcpp::ok() && running_.load()) {
    // Do triggering stuff
    // Check current time - assume ROS uses best clock source
    do {
      if (!rclcpp::ok() || !running_.load()) {
        return;
      }
      now_nsec = rclcpp::Clock{RCL_SYSTEM_TIME}.now().nanoseconds() % (uint64_t)1e9;
      if (now_nsec < end_nsec) {
        while (now_nsec > target_nsec) {
          target_nsec = target_nsec + interval_nsec;
        }
        // FIX: what about very small phases and fast framerates giving a negative number?
        wait_nsec = target_nsec - now_nsec - 1e7;
      } else {
        target_nsec = start_nsec;
        wait_nsec = 1e9 - now_nsec + start_nsec - 1e7;
      }
      // Keep waiting for half the remaining time until the last millisecond.
      // This is required as sleep_for tends to oversleep significantly
      if (wait_nsec > 1e7) {
        rclcpp::sleep_for(std::chrono::nanoseconds(wait_nsec / 2));
      }
    } while (wait_nsec > 1e7);
    if (!rclcpp::ok() || !running_.load()) {
      return;
    }
    // Block the last millisecond
    now_nsec = rclcpp::Clock{RCL_SYSTEM_TIME}.now().nanoseconds() % (uint64_t)1e9;
    if (start_nsec == end_nsec) {
      while (running_.load() && now_nsec > 1e7) {
        now_nsec = rclcpp::Clock{RCL_SYSTEM_TIME}.now().nanoseconds() % (uint64_t)1e9;
      }
    } else if (now_nsec < end_nsec) {
      while (running_.load() && now_nsec < target_nsec) {
        now_nsec = rclcpp::Clock{RCL_SYSTEM_TIME}.now().nanoseconds() % (uint64_t)1e9;
      }
    } else {
      while (running_.load() && (now_nsec > end_nsec || now_nsec < start_nsec)) {
        now_nsec = rclcpp::Clock{RCL_SYSTEM_TIME}.now().nanoseconds() % (uint64_t)1e9;
      }
    }
    if (!rclcpp::ok() || !running_.load()) {
      return;
    }
    // Trigger!
    bool to_high = channel.output.set_level(true);
    rclcpp::sleep_for(std::chrono::nanoseconds(pulse_width));
    rclcpp::Time now = rclcpp::Clock{RCL_SYSTEM_TIME}.now();
    int64_t now_sec = (now.nanoseconds() - pulse_width) / 1e9;  // subtract pulse width
    trigger_time_msg.sec = (int32_t)now_sec;
    trigger_time_msg.nanosec = (uint32_t)now_nsec;
    channel.trigger_time_publisher->publish(trigger_time_msg);
    bool to_low = channel.output.set_level(false);
    target_nsec = target_nsec + interval_nsec >= 1e9 ? start_nsec : target_nsec + interval_nsec;
    if (!(to_high && to_low)) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Failed to set MFP0 level on i2c-" << channel.bus << ": " << strerror(errno));
      rclcpp::shutdown();
      return;
    }
  }
}
}  // namespace sensor_trigger

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_trigger::SensorTrigger)
