/*
Created by clemens on 6/27/18.
This file is part of the software provided by ARTI
Copyright (c) 2018, ARTI
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <vesc_ackermann/axle.h>

namespace vesc_ackermann
{
Axle::Axle(const ros::NodeHandle& nh, const AckermannConfig& common_config, const AxleConfig& axle_config,
           std::shared_ptr<vesc_motor::VescTransportFactory> transport_factory, double position_x)
  : common_config_(common_config), axle_config_(axle_config), position_x_(position_x),
    left_wheel_(position_x, 0.5 * axle_config.track, 0.5 * axle_config.track - axle_config.steering_hinge_offset,
                0.5 * axle_config.wheel_diameter),
    right_wheel_(position_x, -0.5 * axle_config.track, -(0.5 * axle_config.track - axle_config.steering_hinge_offset),
                 0.5 * axle_config.wheel_diameter)
{
  const double execution_duration = 1.0 / (common_config_.odometry_rate * 2.1);

  if (axle_config_.is_steered)
  {
    ros::NodeHandle steering_motor_nh(nh, "steering_motor");
    steering_motor_.emplace(steering_motor_nh, transport_factory, execution_duration,
                            common_config_.publish_motor_speed);
  }

  if (axle_config_.is_driven)
  {
    ros::NodeHandle left_motor_nh(nh, "left_motor");
    left_motor_.emplace(left_motor_nh, transport_factory, execution_duration, common_config_.publish_motor_speed);

    ros::NodeHandle right_motor_nh(nh, "right_motor");
    right_motor_.emplace(right_motor_nh, transport_factory, execution_duration, common_config_.publish_motor_speed);
  }
}

void Axle::setVelocity(const double linear_velocity, const double normalized_steering_angle, const ros::Time& time)
{
  const double tan_normalized_steering_angle = std::tan(normalized_steering_angle);
  const double axle_steering_angle = std::atan2(tan_normalized_steering_angle * position_x_, 1.0);
  const double angular_velocity = tan_normalized_steering_angle * linear_velocity;

  double current_steering_angle = 0.0;
  double current_steering_velocity = 0.0;

  if (steering_motor_)
  {
    current_steering_angle = steering_motor_->getPosition(time);

    const double position_difference = axle_steering_angle - current_steering_angle;
    if (position_difference > common_config_.steering_angle_tolerance)
    {
      current_steering_velocity = common_config_.steering_angle_velocity;
    }
    else if (position_difference < -common_config_.steering_angle_tolerance)
    {
      current_steering_velocity = -common_config_.steering_angle_velocity;
    }

    steering_motor_->setPosition(axle_steering_angle);
  }

  if (left_motor_ && right_motor_)
  {
    const double left_velocity =
      left_wheel_.computeWheelVelocity(linear_velocity, angular_velocity, current_steering_angle,
                                       current_steering_velocity);
    const double right_velocity =
      right_wheel_.computeWheelVelocity(linear_velocity, angular_velocity, current_steering_angle,
                                        current_steering_velocity);

    if ((std::fabs(left_velocity) <= common_config_.brake_velocity)
      && (std::fabs(right_velocity) <= common_config_.brake_velocity)
      && (std::fabs(left_motor_->getVelocity(time)) <= common_config_.allowed_brake_velocity)
      && (std::fabs(right_motor_->getVelocity(time)) <= common_config_.allowed_brake_velocity))
    {
      ROS_DEBUG_STREAM("VescDifferentialDrive::commandVelocityCB::7");

      ROS_DEBUG_STREAM("brake due to left_velocity: " << left_velocity << ", right_velocity: " << right_velocity);
      left_motor_->brake(common_config_.brake_current);
      right_motor_->brake(common_config_.brake_current);
    }
    else
    {
      ROS_DEBUG_STREAM("VescDifferentialDrive::commandVelocityCB::8");

      left_motor_->setVelocity(left_velocity);
      right_motor_->setVelocity(right_velocity);
    }
  }
}

double Axle::getSteeringAngle(const ros::Time& time)
{
  if (steering_motor_)
  {
    return steering_motor_->getPosition(time);
  }
  return 0.0;
}

boost::optional<VehicleVelocity> Axle::getVelocity(const ros::Time& time)
{
  if (left_motor_ && right_motor_)
  {
    const double left_velocity = left_motor_->getVelocity(time);
    const double right_velocity = right_motor_->getVelocity(time);

    if (steering_motor_)
    {
      const double current_steering_angle = steering_motor_->getPosition(time);

      double current_steering_velocity = 0.0;
      if (!last_steering_angle_time_.isZero() && time > last_steering_angle_time_)
      {
        const double time_difference = (time - last_steering_angle_time_).toSec();
        current_steering_velocity = (current_steering_angle - last_steering_angle_) / time_difference;
      }

      VehicleVelocity left_vehicle_velocity
        = left_wheel_.computeVehicleVelocity(left_velocity, current_steering_angle, current_steering_velocity);
      VehicleVelocity right_vehicle_velocity
        = right_wheel_.computeVehicleVelocity(right_velocity, current_steering_angle, current_steering_velocity);

      return VehicleVelocity(
        0.5 * (left_vehicle_velocity.linear_velocity + right_vehicle_velocity.linear_velocity),
        0.5 * (left_vehicle_velocity.angular_velocity + right_vehicle_velocity.angular_velocity)
      );
    }

    // If the axle is not steered, the formula becomes the differential drive formula:
    const double left_tangential_velocity = left_velocity * axle_config_.wheel_diameter * 0.5;
    const double right_tangential_velocity = right_velocity * axle_config_.wheel_diameter * 0.5;

    return VehicleVelocity(
      (left_tangential_velocity + right_tangential_velocity) * 0.5,
      (right_tangential_velocity - left_tangential_velocity) / axle_config_.track
    );
  }
  return boost::none;
}

boost::optional<double> Axle::getSupplyVoltage()
{
  double supply_voltage = 0.0;
  int num_measurements = 0;

  if (left_motor_)
  {
    supply_voltage += left_motor_->getSupplyVoltage();
    ++num_measurements;
  }

  if (right_motor_)
  {
    supply_voltage += right_motor_->getSupplyVoltage();
    ++num_measurements;
  }

  if (steering_motor_)
  {
    supply_voltage += steering_motor_->getSupplyVoltage();
    ++num_measurements;
  }

  if (num_measurements != 0)
  {
    return supply_voltage / num_measurements;
  }
  return boost::none;
}
}
