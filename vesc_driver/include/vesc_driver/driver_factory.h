#ifndef VESC_DRIVER_DRIVER_FACTORY_H
#define VESC_DRIVER_DRIVER_FACTORY_H

#include <chrono>
#include <ros/node_handle.h>
#include <vesc_driver/types.h>
#include <vesc_driver/vesc_driver_interface.h>

namespace vesc_driver
{
class DriverFactory
{
public:
  DriverFactory(TransportFactoryPtr transport_factory, bool use_mockup);

  vesc_driver::VescDriverInterfacePtr createDriver(
    const ros::NodeHandle& nh, std::chrono::duration<double> execution_duration,
    vesc_driver::VescDriverInterface::StateHandlerFunction state_handler_function);

protected:
  TransportFactoryPtr transport_factory_;
  bool use_mockup_;
};
}

#endif //VESC_DRIVER_DRIVER_FACTORY_H
