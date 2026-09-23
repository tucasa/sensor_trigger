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

#ifndef SENSOR_TRIGGER__SENSOR_TRIGGER_HPP_
#define SENSOR_TRIGGER__SENSOR_TRIGGER_HPP_

#include <sensor_trigger/max9296_mfp0.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pthread.h>

#include <array>
#include <atomic>
#include <thread>

namespace sensor_trigger
{
class SensorTrigger : public rclcpp::Node
{
public:
  explicit SensorTrigger(const rclcpp::NodeOptions & node_options);
  ~SensorTrigger();

private:
  struct Mapping
  {
    int bus;
    int camera_a;
    int camera_b;
  };

  struct Channel
  {
    int bus{0};
    int camera_a{0};
    int camera_b{0};
    double phase{0.0};
    int cpu{1};
    bool enabled{false};
    Max9296Mfp0 output;
    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr trigger_time_publisher;
    std::thread trigger_thread;
  };

  // Cameras 0..7 are fixed to these MAX9296 MFP0 inputs. Two cameras share one pin.
  static constexpr int kDeserializerCount = 4;
  static constexpr Mapping kMapping[kDeserializerCount] = {
    {30, 0, 1},
    {31, 2, 3},
    {32, 4, 5},
    {33, 6, 7},
  };

  double fps_{10.0};
  int64_t pulse_width_ms_{5};
  std::atomic<bool> running_{true};
  std::array<Channel, kDeserializerCount> channels_;

  void run(int index);
};
}  // namespace sensor_trigger

#endif  // SENSOR_TRIGGER__SENSOR_TRIGGER_HPP_
