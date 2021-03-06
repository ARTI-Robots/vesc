/*
Created by clemens on 6/13/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef VESC_DRIVER_TRANSPORT_H
#define VESC_DRIVER_TRANSPORT_H

#include <vesc_driver/transport_request.h>

namespace vesc_driver
{
class Transport
{
public:
  typedef std::function<void(const ResponsePacket&)> PacketHandler;
  typedef std::function<void()> TimeoutHandler;

  virtual ~Transport() = default;

  virtual void submit(TransportRequest&& r) = 0;

  virtual void registerPacketHandler(uint8_t controller_id, PacketHandler&& packet_handler) = 0;

  virtual void registerTimeoutHandler(uint8_t controller_id, TimeoutHandler&& timeout_handler) = 0;
};
}

#endif //VESC_DRIVER_TRANSPORT_H
