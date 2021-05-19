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

#include <boost/asio.hpp>

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
  : io_service_(),
    serial_port_(io_service_)
  {}

  void rxThread();

  std::unique_ptr<std::thread> rx_thread_;
  bool rx_thread_run_;
  PacketHandlerFunction packet_handler_;
  ErrorHandlerFunction error_handler_;
  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_port_;
  std::mutex serial_mutex_;
};

void VescInterface::Impl::rxThread()
{
  Buffer buffer;
  buffer.reserve(4096);

  while (rx_thread_run_) {
    int bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;
    if (!buffer.empty()) {
      // search buffer for valid packet(s)
      Buffer::iterator iter(buffer.begin());
      Buffer::iterator iter_begin(buffer.begin());
      while (iter != buffer.end()) {
        // check if valid start-of-frame character
        if (VescFrame::VESC_SOF_VAL_SMALL_FRAME == *iter ||
          VescFrame::VESC_SOF_VAL_LARGE_FRAME == *iter)
        {
          // good start, now attempt to create packet
          std::string error;
          VescPacketConstPtr packet =
            VescPacketFactory::createPacket(iter, buffer.end(), &bytes_needed, &error);
          if (packet) {
            // good packet, check if we skipped any data
            if (std::distance(iter_begin, iter) > 0) {
              std::ostringstream ss;
              ss << "Out-of-sync with VESC, unknown data leading valid frame. Discarding " <<
                std::distance(iter_begin, iter) << " bytes.";
              error_handler_(ss.str());
            }
            // call packet handler
            packet_handler_(packet);
            // update state
            iter = iter + packet->frame().size();
            iter_begin = iter;
            // continue to look for another frame in buffer
            continue;
          } else if (bytes_needed > 0) {
            // need more data, break out of while loop
            break;  // for (iter_sof...
          } else {
            // else, this was not a packet, move on to next byte
            error_handler_(error);
          }
        }

        iter++;
      }

      // if iter is at the end of the buffer, more bytes are needed
      if (iter == buffer.end()) {
        bytes_needed = VescFrame::VESC_MIN_FRAME_SIZE;
      }

      // erase "used" buffer
      if (std::distance(iter_begin, iter) > 0) {
        std::ostringstream ss;
        ss << "Out-of-sync with VESC, discarding " << std::distance(iter_begin, iter) << " bytes.";
        error_handler_(ss.str());
      }
      buffer.erase(buffer.begin(), iter);
    }

    // attempt to read at least bytes_needed bytes from the serial port
    int bytes_to_read = std::max(bytes_needed, 4096);

    {
      std::lock_guard<std::mutex> lock(serial_mutex_);

      const size_t bytes_read = boost::asio::read(
        serial_port_,
        boost::asio::buffer(buffer, buffer.size()),
        boost::asio::transfer_exactly(bytes_to_read));

      if (bytes_needed > 0 && 0 == bytes_read && !buffer.empty()) {
        error_handler_("Possibly out-of-sync with VESC, read timout in the middle of a frame.");
      }
    }

    // Only attempt to read every 10 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

VescInterface::VescInterface(
  const int vesc_id,
  const std::string & port,
  const PacketHandlerFunction & packet_handler,
  const ErrorHandlerFunction & error_handler)
: impl_(new Impl()),
master_vesc_id_(vesc_id)
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
    impl_->serial_port_.open(port);
    impl_->serial_port_.set_option(boost::asio::serial_port_base::baud_rate(115200));
    impl_->serial_port_.set_option(
      boost::asio::serial_port::flow_control(
        boost::asio::serial_port::flow_control::none));
    impl_->serial_port_.set_option(
      boost::asio::serial_port::parity(
        boost::asio::serial_port::parity::none));
    impl_->serial_port_.set_option(
      boost::asio::serial_port::stop_bits(
        boost::asio::serial_port::stop_bits::one));
  } catch (const std::exception & e) {
    std::stringstream ss;
    ss << "Failed to open the serial port to the VESC. " << e.what();
    throw SerialException(ss.str().c_str());
  }

  // start up a monitoring thread
  impl_->rx_thread_run_ = true;
  impl_->rx_thread_ = std::unique_ptr<std::thread>(
    new std::thread(
      &VescInterface::Impl::rxThread, impl_.get()));
}

void VescInterface::disconnect()
{
  // todo - mutex?

  if (isConnected()) {
    // bring down read thread
    impl_->rx_thread_run_ = false;
    impl_->rx_thread_->join();

    std::lock_guard<std::mutex> lock(impl_->serial_mutex_);
    impl_->serial_port_.close();
  }
}

bool VescInterface::isConnected() const
{
  std::lock_guard<std::mutex> lock(impl_->serial_mutex_);
  return impl_->serial_port_.is_open();
}

void VescInterface::send(const VescPacket & packet)
{
  try {
    std::lock_guard<std::mutex> lock(impl_->serial_mutex_);
    size_t written = impl_->serial_port_.write_some(
      boost::asio::buffer(packet.frame()));
    if (written != packet.frame().size()) {
      std::stringstream ss;
      ss << "Wrote " << written << " bytes, expected " << packet.frame().size() << ".";
      throw SerialException(ss.str().c_str());
    }
  } catch (const std::exception & e) {
    std::stringstream ss;
    ss << "Failed to open the serial port to the VESC. " << e.what();
    throw SerialException(ss.str().c_str());
  }
}

void VescInterface::requestFWVersion(int vesc_id)
{

  if (master_vesc_id_==vesc_id || vesc_id==0){
    //ROS_INFO("MASTER"); 
     VescPacketRequestFWVersion pay;
/*
     std::cout << "The vector elements are : "<< pay.frame().size() <<std::endl;
     for(int i=0; i < pay.frame().size(); i++)
         std::cout << std::showbase  << std::hex << std::setw(4) << static_cast<int>(pay.frame().at(i)) << " - ";
     
     std::cout <<std::endl <<"--------------------------------------"<<std::endl;
*/

     send(pay);
  }else{
     //ROS_INFO("FFW");  
     VescPacketCanForwardRequest  pay(vesc_id,VescPacketRequestFWVersion());
/*
     std::cout << "The vector elements are : "<< pay.frame().size() <<std::endl;
     for(int i=0; i < pay.frame().size(); i++)
         std::cout << std::showbase  << std::hex << std::setw(4) << static_cast<int>(pay.frame().at(i)) << " - ";
     
     std::cout <<std::endl <<"--------------------------------------"<<std::endl;
*/
     send(pay);
  }
}

void VescInterface::requestState(int vesc_id)
{
  if (master_vesc_id_==vesc_id || vesc_id==0){
   send(VescPacketRequestValues());
  } else {
   VescPacketCanForwardRequest  pay(vesc_id,VescPacketRequestValues());
    send(pay);
  }  
}

void VescInterface::setDutyCycle(int vesc_id,double duty_cycle)
{
  if (master_vesc_id_==vesc_id || vesc_id==0){
    send(VescPacketSetDuty(duty_cycle));
    } else {
    VescPacketCanForwardRequest  pay(vesc_id,VescPacketSetDuty(duty_cycle));
    send(pay);
  } 
}

void VescInterface::setCurrent(int vesc_id,double current)
{
  if (master_vesc_id_==vesc_id || vesc_id==0){
    send(VescPacketSetCurrent(current));
    } else {
    VescPacketCanForwardRequest  pay(vesc_id,VescPacketSetCurrent(current));
    send(pay);
  } 

  
}

void VescInterface::setBrake(int vesc_id,double brake)
{
  
  if (master_vesc_id_==vesc_id || vesc_id==0){
    send(VescPacketSetCurrentBrake(brake));
    } else {
    VescPacketCanForwardRequest  pay(vesc_id,VescPacketSetCurrentBrake(brake));
    send(pay);
  } 
}

void VescInterface::setSpeed(int vesc_id,double speed)
{
  
  if (master_vesc_id_==vesc_id || vesc_id==0){
    send(VescPacketSetRPM(speed));
    } else {
    VescPacketCanForwardRequest  pay(vesc_id,VescPacketSetRPM(speed));
    send(pay);
  }
}

void VescInterface::setPosition(int vesc_id,double position)
{
  if (master_vesc_id_==vesc_id || vesc_id==0){
    send(VescPacketSetPos(position));
    } else {
    VescPacketCanForwardRequest  pay(vesc_id,VescPacketSetPos(position));
    send(pay);
  }
}

void VescInterface::setServo(int vesc_id,double servo)
{
  if (master_vesc_id_==vesc_id || vesc_id==0){
    send(VescPacketSetServoPos(servo));
    } else {
    VescPacketCanForwardRequest  pay(vesc_id,VescPacketSetServoPos(servo));
    send(pay);
  }
}

}  // namespace vesc_driver
