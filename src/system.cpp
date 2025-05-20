#include "leptrino/system.hpp"

namespace leptrino_force_torque
{
// LeptrinoForceTorqueSensor::~LeptrinoForceTorqueSensor()
// {
//   // If the controller manager is shutdown via Ctrl + C
//   on_cleanup(rclcpp_lifecycle::State());
// }
hardware_interface::CallbackReturn
LeptrinoForceTorqueSensor::on_init(const hardware_interface::HardwareInfo &info)
{
  if (hardware_interface::SensorInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(
      "controller_manager.resource_manager.hardware_component.sensor.ExternalRRBotFTSensor"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_sensor_change_ = stod(info_.hardware_parameters["example_param_max_sensor_change"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  hw_sensor_states_.resize(info_.sensors[0].state_interfaces.size(),
                           std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LeptrinoForceTorqueSensor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // export sensor state interface
  for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
  }

  return state_interfaces;
}

hardware_interface::CallbackReturn
LeptrinoForceTorqueSensor::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
LeptrinoForceTorqueSensor::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  }

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type LeptrinoForceTorqueSensor::read(const rclcpp::Time & /*time*/,
                                                                const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Reading states:";

  for (uint i = 0; i < hw_sensor_states_.size(); i++)
  {
    // Simulate RRBot's sensor data
    unsigned int seed = time(NULL) + i;
    hw_sensor_states_[i] =
        static_cast<float>(rand_r(&seed)) / (static_cast<float>(RAND_MAX / hw_sensor_change_));

    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << hw_sensor_states_[i] << " for sensor '"
       << info_.sensors[0].state_interfaces[i].name.c_str() << "'";
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}
// ----------------------------------------------------------------------------
// Private functions
// ----------------------------------------------------------------------------
void LeptrinoForceTorqueSensor::App_Init()
{
  int rt;

  // Initialize the Comm port
  g_com_ok_ = COM_NG;
  rt = Comm_Open(g_com_port_.c_str());
  if (rt == COM_OK)
  {
    Comm_Setup(460800, PAR_NON, BIT_LEN_8, 0, 0, CHR_ETX);
    g_com_ok_ = COM_OK;
  }
}

void LeptrinoForceTorqueSensor::App_Close(rclcpp::Logger logger)
{
  RCLCPP_DEBUG(logger, "Application close\n");

  if (g_com_ok_ == COM_OK)
  {
    Comm_Close();
  }
}

ULONG LeptrinoForceTorqueSensor::SendData(UCHAR *pucInput, USHORT usSize)
{
  USHORT usCnt;
  UCHAR ucWork;
  UCHAR ucBCC = 0;
  UCHAR *pucWrite = &CommSendBuff_[0];
  USHORT usRealSize;

  // Reformat the data
  *pucWrite = CHR_DLE; // DLE
  pucWrite++;
  *pucWrite = CHR_STX; // STX
  pucWrite++;
  usRealSize = 2;

  for (usCnt = 0; usCnt < usSize; usCnt++)
  {
    ucWork = pucInput[usCnt];
    if (ucWork == CHR_DLE)
    {                      // if data is 0x10 then add 0x10
      *pucWrite = CHR_DLE; // DLE
      pucWrite++;          // writing destination
      usRealSize++;        // actual size
      // Do not calculate BCC!
    }
    *pucWrite = ucWork; // data
    ucBCC ^= ucWork;    // BCC
    pucWrite++;         // writing destination
    usRealSize++;       // actual size
  }

  *pucWrite = CHR_DLE; // DLE
  pucWrite++;
  *pucWrite = CHR_ETX; // ETX
  ucBCC ^= CHR_ETX;    // BCC
  pucWrite++;
  *pucWrite = ucBCC; // BCC
  usRealSize += 3;

  Comm_SendData(&CommSendBuff_[0], usRealSize);

  return COM_OK;
}

void LeptrinoForceTorqueSensor::GetProductInfo(rclcpp::Logger logger)
{
  USHORT len;

  RCLCPP_INFO(logger, "Get sensor information");
  len = 0x04;                 // data length
  SendBuff_[0] = len;         // length
  SendBuff_[1] = 0xFF;        // sensor no.
  SendBuff_[2] = CMD_GET_INF; // command type
  SendBuff_[3] = 0;           // reserve

  SendData(SendBuff_, len);
}

void LeptrinoForceTorqueSensor::GetLimit(rclcpp::Logger logger)
{
  USHORT len;

  RCLCPP_INFO(logger, "Get sensor limit");
  len = 0x04;                   // data length
  SendBuff_[0] = len;           // length
  SendBuff_[1] = 0xFF;          // sensor no.
  SendBuff_[2] = CMD_GET_LIMIT; // command type
  SendBuff_[3] = 0;             // reserve

  SendData(SendBuff_, len);
}

void LeptrinoForceTorqueSensor::SerialStart(rclcpp::Logger logger)
{
  USHORT len;

  RCLCPP_INFO(logger, "Start sensor");
  len = 0x04;                    // data length
  SendBuff_[0] = len;            // length
  SendBuff_[1] = 0xFF;           // sensor no.
  SendBuff_[2] = CMD_DATA_START; // command type
  SendBuff_[3] = 0;              // reserve

  SendData(SendBuff_, len);
}

void LeptrinoForceTorqueSensor::SerialStop(rclcpp::Logger logger)
{
  USHORT len;

  RCLCPP_INFO(logger, "Stop sensor");
  len = 0x04;                   // data length
  SendBuff_[0] = len;           // length
  SendBuff_[1] = 0xFF;          // sensor no.
  SendBuff_[2] = CMD_DATA_STOP; // command type
  SendBuff_[3] = 0;             // reserve

  SendData(SendBuff_, len);
}

} // namespace leptrino_force_torque

// ----------------------------------------------------------------------------
// Export plugin
// ----------------------------------------------------------------------------
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(leptrino_force_torque::LeptrinoForceTorqueSensor,
                       hardware_interface::SensorInterface)