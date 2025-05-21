#ifndef _LEPTRINO_SYSTEM_HPP
#define _LEPTRINO_SYSTEM_HPP

#include <hardware_interface/sensor_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "leptrino/pComResInternal.h"
#include "leptrino/pCommon.h"
#include "leptrino/rs_comm.h"

namespace leptrino_force_torque
{
class LeptrinoForceTorqueSensor : public hardware_interface::SensorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LeptrinoForceTorqueSensor)

  // ~LeptrinoForceTorqueSensor() override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  /// Get the logger of the SensorInterface.
  /**
   * \return logger of the SensorInterface.
   */
  rclcpp::Logger get_logger() const { return *logger_; }

  /// Get the clock of the SensorInterface.
  /**
   * \return clock of the SensorInterface.
   */
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

private:
  // Communication variables
  std::string g_com_port_;
  int g_rate_;
  int g_com_ok_;
  UCHAR CommRcvBuff_[256];
  UCHAR CommSendBuff_[1024];
  UCHAR SendBuff_[512];

  // Calibration variables
  double conversion_factor_[FN_Num];

  // functions
  void App_Init();
  void App_Close(rclcpp::Logger logger);
  ULONG SendData(UCHAR *pucInput, USHORT usSize);
  void GetProductInfo(rclcpp::Logger logger);
  void GetLimit(rclcpp::Logger logger);
  void SerialStart(rclcpp::Logger logger);
  void SerialStop(rclcpp::Logger logger);

  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  // Store the sensor states for the simulated robot
  std::vector<double> hw_sensor_states_;
};
} // namespace leptrino_force_torque
#endif // _LEPTRINO_SYSTEM_HPP