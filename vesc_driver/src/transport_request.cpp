/*
Created by clemens on 6/13/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <vesc_driver/transport_request.h>
#include <ros/ros.h>

namespace vesc_driver
{
  TransportRequest::TransportRequest(uint8_t controller_id, vesc_driver::PacketVariant &&packet, bool expect_response)
      : controller_id_(controller_id), packet_(std::forward(packet)), expect_response_(expect_response)
  {
    ROS_DEBUG_STREAM("TransportRequest::TransportRequest packet type: " << packet_.type().name());
  }

  uint8_t TransportRequest::getControllerId() const
  {
    return controller_id_;
  }

  const PacketVariant& TransportRequest::getPacket() const
  {
    return packet_;
  }

  bool TransportRequest::expectResponse() const
  {
    return expect_response_;
  }
}
