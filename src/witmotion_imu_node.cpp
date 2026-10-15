// Copyright (c) 2026 Masazumi Imai
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

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace witmotion_imu_driver
{

class Driver : public rclcpp::Node
{
public:
  Driver() : Node("witmotion_imu")
  {
    rcl_interfaces::msg::ParameterDescriptor startup_descriptor;
    startup_descriptor.read_only = true;
    const auto device =
      declare_parameter("device", rclcpp::ParameterType::PARAMETER_STRING, startup_descriptor)
        .get<std::string>();
    const auto baud =
      declare_parameter("baud_rate", rclcpp::ParameterType::PARAMETER_INTEGER, startup_descriptor)
        .get<int64_t>();
    frame_ =
      declare_parameter("frame_id", rclcpp::ParameterType::PARAMETER_STRING, startup_descriptor)
        .get<std::string>();
    const double timeout =
      declare_parameter("timeout", rclcpp::ParameterType::PARAMETER_DOUBLE, startup_descriptor)
        .get<double>();
    const double rate =
      declare_parameter("update_rate", rclcpp::ParameterType::PARAMETER_DOUBLE, startup_descriptor)
        .get<double>();
    const bool use_6axis =
      declare_parameter("use_6axis", rclcpp::ParameterType::PARAMETER_BOOL, startup_descriptor)
        .get<bool>();
    // NOTE: these codes match the tested wired firmware.
    constexpr std::array<double, 8> rates{1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 200.0};
    constexpr std::array<uint16_t, 8> rate_codes{0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0b};
    std::size_t rate_index = 0;
    while (rate_index < rates.size() && rates[rate_index] != rate) ++rate_index;
    if (
      device.empty() || frame_.empty() || baud != 115200 || !std::isfinite(timeout) ||
      timeout <= 0.0 || timeout > 60.0) {
      throw std::invalid_argument(
        "Require a device, frame_id, baud_rate=115200 and timeout in (0, 60] s");
    }
    if (rate_index == rates.size() || timeout < 2.0 / rate) {
      throw std::invalid_argument(
        "update_rate must be 1, 2, 5, 10, 20, 50, 100 or 200 Hz; timeout must be >= 2/update_rate");
    }
    timeout_ = std::chrono::duration<double>(timeout);
    publisher_ = create_publisher<sensor_msgs::msg::Imu>("data_raw", rclcpp::SensorDataQoS());
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) fail("Cannot open " + device);
    try {
      if (::ioctl(fd_, TIOCEXCL) < 0) fail("Cannot lock serial port");
      if (::tcgetattr(fd_, &original_) < 0) fail("Cannot read serial settings");
      has_original_ = true;
      termios settings = original_;
      ::cfmakeraw(&settings);
      settings.c_iflag &= ~(IXON | IXOFF | IXANY);
      settings.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
      settings.c_cflag |= CS8 | CLOCAL | CREAD;
      settings.c_cc[VMIN] = settings.c_cc[VTIME] = 0;
      if (
        ::cfsetispeed(&settings, B115200) < 0 || ::cfsetospeed(&settings, B115200) < 0 ||
        ::tcsetattr(fd_, TCSANOW, &settings) < 0 || ::tcflush(fd_, TCIFLUSH) < 0) {
        fail("Cannot configure serial port");
      }
      last_sample_ = std::chrono::steady_clock::now();
      writeRegister(0x03, rate_codes[rate_index]);
      if (readRegister(0x03) != rate_codes[rate_index]) {
        throw std::runtime_error("Sensor update_rate readback failed or did not match the request");
      }
      const uint16_t axis_mode = use_6axis ? 0x01 : 0x00;
      writeRegister(0x24, axis_mode);
      if (readRegister(0x24) != axis_mode) {
        throw std::runtime_error("Sensor use_6axis readback failed or did not match the request");
      }
      configured_ = true;
      last_sample_ = std::chrono::steady_clock::now();
      timer_ = create_wall_timer(std::chrono::milliseconds(1), [this] { receive(); });
      start_magnetic_calibration_service_ = create_service<std_srvs::srv::Trigger>(
        "~/start_magnetic_calibration",
        [this](
          const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          calibrateMagneticField(true, *response);
        });
      stop_magnetic_calibration_service_ = create_service<std_srvs::srv::Trigger>(
        "~/stop_magnetic_calibration",
        [this](
          const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          calibrateMagneticField(false, *response);
        });
    } catch (...) {
      closePort();
      throw;
    }
    RCLCPP_INFO(
      get_logger(),
      "Reading WT901BLECL on %s at 115200 bps, sensor rate %.1f Hz, %s mode confirmed; data_raw in "
      "%s",
      device.c_str(), rate, use_6axis ? "6-axis" : "9-axis", frame_.c_str());
  }

  ~Driver() override { closePort(); }

private:
  void receiveFor(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      receive();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  void writeRegister(uint8_t address, uint16_t value)
  {
    sendCommand(
      0x69, 0xb588);  // Unlock before each write; calibration can exceed the unlock period.
    receiveFor(std::chrono::milliseconds(100));
    sendCommand(address, value);
    receiveFor(std::chrono::milliseconds(100));
  }

  int readRegister(uint8_t address)
  {
    readback_address_ = address;
    register_readback_ = -1;
    sendCommand(0x27, address);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (register_readback_ < 0 && std::chrono::steady_clock::now() < deadline) {
      receive();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (register_readback_ < 0) {
      throw std::runtime_error("Sensor register readback timed out");
    }
    return register_readback_;
  }

  void calibrateMagneticField(bool start, std_srvs::srv::Trigger::Response & response)
  {
    try {
      const int current_mode = readRegister(0x01);
      if (start && current_mode == 0x07) {
        response.message =
          "Magnetic calibration is already active; rotate the sensor, then call stop.";
        return;
      }
      if (current_mode != 0x00 && current_mode != 0x07) {
        throw std::runtime_error(
          "Sensor is in another calibration mode; magnetic calibration refused");
      }
      const uint16_t requested_mode = start ? 0x07 : 0x00;
      writeRegister(0x01, requested_mode);
      if (readRegister(0x01) != requested_mode) {
        throw std::runtime_error(
          "Calibration mode readback did not match; call stop to restore normal mode");
      }
      if (!start) {
        // Saving persists all current settings, including update_rate, not only magnetic calibration.
        writeRegister(0x00, 0x00);
      }
      response.success = true;
      // NOTE: CALSW confirms the mode, not calibration quality or EEPROM persistence.
      // Verify yaw after calibration and power cycling; the save command has no dedicated acknowledgement.
      response.message = start ? "Magnetic calibration mode confirmed. Slowly rotate 360 degrees "
                                 "around X, Y and Z, then call stop."
                               : "Normal mode confirmed; settings save command sent. Check yaw "
                                 "stability before using the attitude.";
      RCLCPP_INFO(get_logger(), "%s", response.message.c_str());
    } catch (const std::exception & error) {
      response.success = false;
      response.message = error.what();
      RCLCPP_ERROR(get_logger(), "Magnetic calibration failed: %s", error.what());
    }
  }

  void sendCommand(uint8_t address, uint16_t value)
  {
    const std::array<uint8_t, 5> command{
      0xff, 0xaa, address, static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8)};
    std::size_t sent = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (sent < command.size()) {
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("Serial command write timed out");
      }
      const ssize_t count = ::write(fd_, command.data() + sent, command.size() - sent);
      if (count > 0) {
        sent += static_cast<std::size_t>(count);
      } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        fail("Serial command write failed");
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

  [[noreturn]] static void fail(const std::string & operation)
  {
    throw std::runtime_error(operation + ": " + std::strerror(errno));
  }

  void closePort() noexcept
  {
    if (fd_ < 0) return;
    if (has_original_) (void)::tcsetattr(fd_, TCSANOW, &original_);
    (void)::ioctl(fd_, TIOCNXCL);
    (void)::close(fd_);
    fd_ = -1;
  }

  static int signedWord(const uint8_t * bytes)
  {
    const int value = bytes[0] | (static_cast<int>(bytes[1]) << 8);
    return value >= 32768 ? value - 65536 : value;
  }

  void push(uint8_t byte)
  {
    if (size_ == 0) {
      if (byte == 0x55) packet_[size_++] = byte;
      return;
    }
    if (size_ == 1 && byte != 0x61 && byte != 0x71) {
      size_ = byte == 0x55 ? 1 : 0;
      return;
    }
    packet_[size_++] = byte;
    if (size_ != packet_.size()) return;
    size_ = 0;
    if (packet_[1] == 0x71) {
      if ((packet_[2] | (static_cast<uint16_t>(packet_[3]) << 8)) == readback_address_) {
        register_readback_ = packet_[4] | (static_cast<int>(packet_[5]) << 8);
      }
      return;
    }
    last_sample_ = std::chrono::steady_clock::now();
    if (!configured_) return;

    constexpr double pi = 3.14159265358979323846;
    constexpr double acceleration_scale = 16.0 * 9.80665 / 32768.0;
    constexpr double angular_velocity_scale = 2000.0 * pi / (180.0 * 32768.0);
    constexpr double angle_scale = pi / 32768.0;
    sensor_msgs::msg::Imu message;
    message.header.frame_id = frame_;
    // NOTE: this protocol has no sample clock; USB batching is represented by host decode
    // timestamps. Use device timestamps if a future protocol exposes them.
    message.header.stamp = now();
    message.linear_acceleration.x = signedWord(packet_.data() + 2) * acceleration_scale;
    message.linear_acceleration.y = signedWord(packet_.data() + 4) * acceleration_scale;
    message.linear_acceleration.z = signedWord(packet_.data() + 6) * acceleration_scale;
    message.angular_velocity.x = signedWord(packet_.data() + 8) * angular_velocity_scale;
    message.angular_velocity.y = signedWord(packet_.data() + 10) * angular_velocity_scale;
    message.angular_velocity.z = signedWord(packet_.data() + 12) * angular_velocity_scale;
    tf2::Quaternion orientation;
    orientation.setRPY(
      signedWord(packet_.data() + 14) * angle_scale, signedWord(packet_.data() + 16) * angle_scale,
      signedWord(packet_.data() + 18) * angle_scale);
    orientation.normalize();
    message.orientation.x = orientation.x();
    message.orientation.y = orientation.y();
    message.orientation.z = orientation.z();
    message.orientation.w = orientation.w();
    // Zero covariance means unknown; the device does not provide uncertainty estimates.
    publisher_->publish(message);
  }

  void receive()
  {
    std::array<uint8_t, 512> bytes;
    // Bound work per callback even if a faulty device floods the port.
    for (int attempt = 0; attempt < 8; ++attempt) {
      pollfd descriptor{fd_, POLLIN, 0};
      const int ready = ::poll(&descriptor, 1, 0);
      if (ready < 0) {
        if (errno == EINTR) break;
        fail("Serial poll failed");
      }
      if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        throw std::runtime_error("Serial device disconnected");
      }
      if (!(descriptor.revents & POLLIN)) break;
      const ssize_t count = ::read(fd_, bytes.data(), bytes.size());
      if (count < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) break;
        fail("Serial read failed");
      }
      if (count == 0) break;
      for (ssize_t i = 0; i < count; ++i) push(bytes[static_cast<std::size_t>(i)]);
    }
    if (std::chrono::steady_clock::now() - last_sample_ > timeout_) {
      throw std::runtime_error("No complete WT901BLECL IMU packet within timeout");
    }
    // NOTE: the 20-byte protocol has no checksum. Header resynchronization cannot detect
    // all corrupted payloads; use integrity-protected device output if this becomes necessary.
  }

  int fd_{-1};
  int register_readback_{-1};
  uint16_t readback_address_{0xffff};
  bool configured_{false};
  bool has_original_{false};
  termios original_{};
  std::array<uint8_t, 20> packet_{};
  std::size_t size_{0};
  std::string frame_;
  std::chrono::steady_clock::time_point last_sample_;
  std::chrono::duration<double> timeout_{};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_magnetic_calibration_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_magnetic_calibration_service_;
};

}  // namespace witmotion_imu_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<witmotion_imu_driver::Driver>());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "witmotion_imu_driver: %s\n", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
