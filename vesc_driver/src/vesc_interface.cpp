// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver/vesc_interface.hpp"
#include "vesc_driver/vesc_packet_factory.hpp"
#include "serial_driver/serial_driver.hpp"


#include <algorithm>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace vesc_driver
{

class VescInterface::Impl
{
public:
  Impl()
  : owned_ctx{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx)}
  {}

  void serial_receive_callback(const std::vector<uint8_t> & buffer);
  void packet_creation_thread();
  void on_configure();
  void connect(const std::string & port);

  bool packet_thread_run_;
  std::unique_ptr<std::thread> packet_thread_;
  PacketHandlerFunction packet_handler_;
  ErrorHandlerFunction error_handler_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::mutex buffer_mutex_;
  std::string device_name_;
  std::unique_ptr<IoContext> owned_ctx{};
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  
  ~Impl(){
    if (owned_ctx) {
      owned_ctx->waitForExit();
  }
  }
private:
  std::vector<uint8_t> buffer_;
};

void VescInterface::Impl::serial_receive_callback(const std::vector<uint8_t> & buffer)
{
  buffer_mutex_.lock(); // wait untill the current buffer read finishes
  buffer_.reserve(buffer_.size() + buffer.size());
  buffer_.insert(buffer_.end(), buffer.begin(), buffer.end());
  buffer_mutex_.unlock();
}

void VescInterface::Impl::packet_creation_thread()
{
  while (packet_thread_run_)
  {
    buffer_mutex_.lock();
    int bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;
    if (!buffer_.empty()) {
      // search buffer for valid packet(s)
      auto iter = buffer_.begin();
      while (iter != buffer_.end())
      {
        // check if valid start-of-frame character
        if (VescFrame::VESC_SOF_VAL_SMALL_FRAME == *iter ||
          VescFrame::VESC_SOF_VAL_LARGE_FRAME == *iter)
        {
          // good start, now attempt to create packet
          std::string error;
          VescPacketConstPtr packet =
            VescPacketFactory::createPacket(iter, buffer_.end(), &bytes_needed, &error);
          if (packet) {
            // call packet handler
            packet_handler_(packet);
            // update state
            iter = iter + packet->frame().size();
            // continue to look for another frame in buffer
            continue;
          } else if (bytes_needed > 0) {
            // need more data, break out of while loop
            break;  // for (iter_sof...
          }
        }

        iter++;
      }

      // if iter is at the end of the buffer, more bytes are needed
      if (iter == buffer_.end()) {
        bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;
      }

      // erase "used" buffer
      buffer_.erase(buffer_.begin(), iter);
    }

    buffer_mutex_.unlock();
    // Only attempt to read every 10 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void VescInterface::Impl::connect(const std::string & port)
{
  uint32_t baud_rate = 115200;
  auto fc = drivers::serial_driver::FlowControl::HARDWARE;
  auto pt = drivers::serial_driver::Parity::NONE;
  auto sb = drivers::serial_driver::StopBits::ONE;
  device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
  serial_driver_->init_port(port, *device_config_);
  if (!serial_driver_->port()->is_open()) {
    serial_driver_->port()->open();
    serial_driver_->port()->async_receive(
      std::bind(&VescInterface::Impl::serial_receive_callback, this, std::placeholders::_1));
  }

}

VescInterface::VescInterface(
  const std::string & port,
  const PacketHandlerFunction & packet_handler,
  const ErrorHandlerFunction & error_handler)
: impl_(new Impl())
{
  setPacketHandler(packet_handler);
  setErrorHandler(error_handler);
  // attempt to conect if the port is specified
  if (!port.empty()) {
    connect(port);
  }
}

VescInterface::~VescInterface()
{
  disconnect();
}

void VescInterface::setPacketHandler(const PacketHandlerFunction & handler)
{
  // todo - definately need mutex
  impl_->packet_handler_ = handler;
}

void VescInterface::setErrorHandler(const ErrorHandlerFunction & handler)
{
  // todo - definately need mutex
  impl_->error_handler_ = handler;
}

void VescInterface::connect(const std::string & port)
{
  // todo - mutex?

  if (isConnected()) {
    throw SerialException("Already connected to serial port.");
  }

  // connect to serial port
  try {
    impl_->connect(port);
  } catch (const std::exception & e) {
    std::stringstream ss;
    ss << "Failed to open the serial port " << port << " to the VESC. " << e.what();
    throw SerialException(ss.str().c_str());
  }

  // start up a monitoring thread
  impl_->packet_thread_run_ = true;
  impl_->packet_thread_ = std::unique_ptr<std::thread>(
    new std::thread(
      &VescInterface::Impl::packet_creation_thread, impl_.get()));
}

void VescInterface::disconnect()
{
  // todo - mutex?

  if (isConnected()) {
    // bring down read thread
    impl_->packet_thread_run_ = false;
    impl_->packet_thread_->join();
    impl_->serial_driver_->port()->close();
  }
}

bool VescInterface::isConnected() const
{
  auto port = impl_->serial_driver_->port();
  if (port)
  {
    return port->is_open();
  }
  else
  {
    return false;
  }
}

void VescInterface::send(const VescPacket & packet)
{
  impl_->serial_driver_->port()->async_send(packet.frame());
}

void VescInterface::requestFWVersion()
{
  send(VescPacketRequestFWVersion());
}

void VescInterface::requestState()
{
  send(VescPacketRequestValues());
}

void VescInterface::setDutyCycle(double duty_cycle)
{
  send(VescPacketSetDuty(duty_cycle));
}

void VescInterface::setCurrent(double current)
{
  send(VescPacketSetCurrent(current));
}

void VescInterface::setBrake(double brake)
{
  send(VescPacketSetCurrentBrake(brake));
}

void VescInterface::setSpeed(double speed)
{
  send(VescPacketSetRPM(speed));
}

void VescInterface::setPosition(double position)
{
  send(VescPacketSetPos(position));
}

void VescInterface::setServo(double servo)
{
  send(VescPacketSetServoPos(servo));
}

}  // namespace vesc_driver
