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

#include <sensor_trigger/max9296_mfp0.hpp>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <string>

namespace sensor_trigger
{
Max9296Mfp0::~Max9296Mfp0()
{
  if (configured_) {
    restore();
  }
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
}

bool Max9296Mfp0::open(int bus)
{
  const std::string path = "/dev/i2c-" + std::to_string(bus);
  fd_ = ::open(path.c_str(), O_RDWR);
  if (fd_ < 0) {
    return false;
  }
  if (
    !read_reg(kGpioA, saved_gpio_a_) || !read_reg(kGpioB, saved_gpio_b_) ||
    !read_reg(kGpioC, saved_gpio_c_)) {
    return false;
  }
  // Saved registers are valid. The destructor writes them back if setup fails.
  configured_ = true;
  if (!write_reg(kGpioB, kTxId) || !write_reg(kGpioA, kLevelLow)) {
    return false;
  }
  return true;
}

bool Max9296Mfp0::set_level(bool high)
{
  return write_reg(kGpioA, high ? kLevelHigh : kLevelLow);
}

bool Max9296Mfp0::read_reg(uint16_t reg, uint8_t & value)
{
  uint8_t address[2] = {
    static_cast<uint8_t>((reg >> 8) & 0xff), static_cast<uint8_t>(reg & 0xff)};
  uint8_t data = 0;
  struct i2c_msg msgs[2];
  msgs[0].addr = kSlaveAddress;
  msgs[0].flags = 0;
  msgs[0].len = 2;
  msgs[0].buf = address;
  msgs[1].addr = kSlaveAddress;
  msgs[1].flags = I2C_M_RD;
  msgs[1].len = 1;
  msgs[1].buf = &data;

  struct i2c_rdwr_ioctl_data transfer;
  transfer.msgs = msgs;
  transfer.nmsgs = 2;
  if (ioctl(fd_, I2C_RDWR, &transfer) < 0) {
    return false;
  }
  value = data;
  return true;
}

bool Max9296Mfp0::write_reg(uint16_t reg, uint8_t value)
{
  uint8_t buffer[3] = {
    static_cast<uint8_t>((reg >> 8) & 0xff), static_cast<uint8_t>(reg & 0xff), value};
  struct i2c_msg msg;
  msg.addr = kSlaveAddress;
  msg.flags = 0;
  msg.len = 3;
  msg.buf = buffer;

  struct i2c_rdwr_ioctl_data transfer;
  transfer.msgs = &msg;
  transfer.nmsgs = 1;
  return ioctl(fd_, I2C_RDWR, &transfer) >= 0;
}

void Max9296Mfp0::restore()
{
  write_reg(kGpioA, saved_gpio_a_);
  write_reg(kGpioB, saved_gpio_b_);
  write_reg(kGpioC, saved_gpio_c_);
  configured_ = false;
}
}  // namespace sensor_trigger
