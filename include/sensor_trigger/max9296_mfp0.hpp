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

#ifndef SENSOR_TRIGGER__MAX9296_MFP0_HPP_
#define SENSOR_TRIGGER__MAX9296_MFP0_HPP_

#include <cstdint>

namespace sensor_trigger
{
// Drives MAX9296 MFP0 through register 0x02B0. High is 0x12 and Low is 0x02.
// UART1 (register 0x0003) is left unchanged.
class Max9296Mfp0
{
public:
  Max9296Mfp0() = default;
  ~Max9296Mfp0();
  Max9296Mfp0(const Max9296Mfp0 &) = delete;
  Max9296Mfp0 & operator=(const Max9296Mfp0 &) = delete;

  bool open(int bus);
  bool set_level(bool high);

private:
  bool read_reg(uint16_t reg, uint8_t & value);
  bool write_reg(uint16_t reg, uint8_t value);
  void restore();

  static constexpr int kSlaveAddress = 0x48;
  static constexpr uint16_t kGpioA = 0x02B0;
  static constexpr uint16_t kGpioB = 0x02B1;
  static constexpr uint16_t kGpioC = 0x02B2;
  static constexpr uint8_t kLevelLow = 0x02;
  static constexpr uint8_t kLevelHigh = 0x12;
  static constexpr uint8_t kTxId = 0x06;

  int fd_{-1};
  bool configured_{false};
  uint8_t saved_gpio_a_{0};
  uint8_t saved_gpio_b_{0};
  uint8_t saved_gpio_c_{0};
};
}  // namespace sensor_trigger

#endif  // SENSOR_TRIGGER__MAX9296_MFP0_HPP_
