#ifndef VESC_DRIVER_TRANSPORT_FACTORY_H
#define VESC_DRIVER_TRANSPORT_FACTORY_H

#include <cstdint>
#include <map>
#include <ros/node_handle.h>
#include <string>
#include <vesc_driver/types.h>

namespace vesc_driver
{
class TransportFactory
{
public:
  explicit TransportFactory(const ros::NodeHandle& nh, const std::string& parameter_name = "transport_mapping");

  /**
   * Returns a transport by name.
   *
   * @throw std::invalid_argument if no transport with the given name exists.
   */
  vesc_driver::TransportPtr getTransport(const std::string& name);

  /**
   * Creates a transport that connects to a VESC via serial port.
   *
   * @param controller_id the ID of the connected VESC. When messages to other IDs are sent over this transport, they
   *                      will be forwarded via CAN.
   * @param port the name of the serial port.
   * @return the (already connected) transport.
   */
  vesc_driver::TransportPtr createSerialTransport(uint8_t controller_id, const std::string& port);

protected:
  typedef std::map<std::string, vesc_driver::TransportPtr> TransportMap;

  template<typename T>
  static T getRequiredParameter(XmlRpc::XmlRpcValue& transport_mapping, const std::string& parameter_name);

  ros::NodeHandle nh_;
  std::string parameter_name_;
  TransportMap transport_map_;
};
}

#endif //VESC_DRIVER_TRANSPORT_FACTORY_H
