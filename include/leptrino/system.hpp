#ifndef _LEPTRINO_SYSTEM_HPP
#define _LEPTRINO_SYSTEM_HPP

#include <hardware_interface/sensor_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include "leptrino/rs_comm.h"
#include "leptrino/pComResInternal.h"
#include "leptrino/pCommon.h"

class LeptrinoForceTorqueSensor : public hardware_interface::SensorInterface
{
public:
  ~LeptrinoForceTorqueSensor() override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  std::string g_com_port_;
  int g_rate_;
  int g_com_ok_;
  UCHAR CommRcvBuff_[256];
  UCHAR CommSendBuff_[1024];
  UCHAR SendBuff_[512];
  double conversion_factor_[FN_Num];

  // functions
  void App_Init();
  void App_Close(rclcpp::Logger logger);
  ULONG SendData(UCHAR *pucInput, USHORT usSize);
  void GetProductInfo(rclcpp::Logger logger);
  void GetLimit(rclcpp::Logger logger);
  void SerialStart(rclcpp::Logger logger);
  void SerialStop(rclcpp::Logger logger);

  // state interfaces
  std::vector<double> hw_sensor_states_;
};

#endif // _LEPTRINO_SYSTEM_HPP