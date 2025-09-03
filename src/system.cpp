// TODO: Not threaded safe!

#include <chrono>

#include "leptrino/system.hpp"

namespace leptrino_force_torque
{
LeptrinoForceTorqueSensor::~LeptrinoForceTorqueSensor()
{
  // Best-effort cleanup if destructed without lifecycle transitions
  try
  {
    SerialStop(rclcpp::get_logger("LeptrinoForceTorqueSensor"));
    App_Close(rclcpp::get_logger("LeptrinoForceTorqueSensor"));
  }
  catch (...)
  {
  }
}
hardware_interface::CallbackReturn
LeptrinoForceTorqueSensor::on_init(const hardware_interface::HardwareInfo &info)
{
  // Initialize the sensor interface
  if (SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Validate hardware info
  if (info_.sensors.empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("LeptrinoForceTorqueSensor"),
                 "No sensors defined in hardware info");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.sensors[0].state_interfaces.size() != static_cast<size_t>(FN_Num))
  {
    RCLCPP_ERROR(rclcpp::get_logger("LeptrinoForceTorqueSensor"),
                 "State interface count (%zu) does not match expected FN_Num (%d)",
                 info_.sensors[0].state_interfaces.size(), FN_Num);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read the parameters from the hardware interface
  if (info_.hardware_parameters.find("com_port") != info_.hardware_parameters.end())
  {
    g_com_port_ = info_.hardware_parameters.at("com_port");
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("LeptrinoForceTorqueSensor"),
                "Port is not defined, trying /dev/ttyACM0");
    g_com_port_ = "/dev/ttyACM0";
  }

  if (info_.hardware_parameters.find("rate") != info_.hardware_parameters.end())
  {
    g_rate_ = std::stoi(info_.hardware_parameters.at("rate"));
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("LeptrinoForceTorqueSensor"),
                "Rate is not defined, using maximum 1.2 kHz");
    g_rate_ = 1200;
  }

  if (info_.hardware_parameters.find("calib_len") != info_.hardware_parameters.end())
  {
    calib_len_ = std::stoi(info_.hardware_parameters.at("calib_len"));
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("LeptrinoForceTorqueSensor"),
                "Calibration length is not defined, using no calibration");
    calib_len_ = -1;
  }

  // Initialize the calibration offset
  calib_offset_.clear();
  calib_offset_.resize(FN_Num, 0.0);

  // Initialize the object for logging
  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("LeptrinoForceTorqueSensor"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // Initialize the state interfaces
  hw_sensor_states_.resize(info_.sensors[0].state_interfaces.size(),
                           std::numeric_limits<double>::quiet_NaN());

  // Initialize conversion factors to safe defaults
  for (int i = 0; i < FN_Num; ++i)
  {
    conversion_factor_[i] = 1.0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> LeptrinoForceTorqueSensor::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // export sensor state interface
  for (size_t i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
  }

  return state_interfaces;
}

hardware_interface::CallbackReturn
LeptrinoForceTorqueSensor::on_configure(const rclcpp_lifecycle::State &previous_state)
{
  // Initialize the application
  App_Init();
  if (g_com_ok_ == COM_NG)
  {
    // Drain any stale data that might be in the device buffer (previous-run leftovers)
    DrainRx(std::chrono::milliseconds(50));
    RCLCPP_ERROR(rclcpp::get_logger("LeptrinoForceTorqueSensor"), "Failed to open the port %s",
                 g_com_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get the product information with timeout
  GetProductInfo(rclcpp::get_logger("LeptrinoForceTorqueSensor"));
  using clock = std::chrono::steady_clock;
  const auto timeout = std::chrono::seconds(2);
  auto start = clock::now();
  rclcpp::Rate wait_rate(g_rate_);
  bool got_info = false;
  while (rclcpp::ok() && (clock::now() - start) < timeout)
  {
    Comm_Rcv();
    if (Comm_CheckRcv() != 0)
    { // Receive data
      CommRcvBuff_[0] = 0;

      auto rt = Comm_GetRcvData(CommRcvBuff_);
      if (rt > 0)
      {
        auto stGetInfo = (ST_R_GET_INF *)CommRcvBuff_;
        stGetInfo->scFVer[F_VER_SIZE] = 0;
        RCLCPP_INFO(rclcpp::get_logger("LeptrinoForceTorqueSensor"), "Version: %s",
                    stGetInfo->scFVer);
        stGetInfo->scSerial[SERIAL_SIZE] = 0;
        RCLCPP_INFO(rclcpp::get_logger("LeptrinoForceTorqueSensor"), "SerialNo: %s",
                    stGetInfo->scSerial);
        stGetInfo->scPName[P_NAME_SIZE] = 0;
        RCLCPP_INFO(rclcpp::get_logger("LeptrinoForceTorqueSensor"), "Type: %s",
                    stGetInfo->scPName);
        got_info = true;
        break;
      }
    }
    wait_rate.sleep();
  }
  if (!got_info)
  {
    RCLCPP_ERROR(rclcpp::get_logger("LeptrinoForceTorqueSensor"),
                 "Timeout while getting product information");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get the limit information with timeout
  GetLimit(rclcpp::get_logger("LeptrinoForceTorqueSensor"));
  start = clock::now();
  bool got_limit = false;
  while (rclcpp::ok() && (clock::now() - start) < timeout)
  {
    Comm_Rcv();
    if (Comm_CheckRcv() != 0)
    { // Receive data
      CommRcvBuff_[0] = 0;

      auto rt = Comm_GetRcvData(CommRcvBuff_);
      if (rt > 0)
      {
        auto stGetLimit = (ST_R_LEP_GET_LIMIT *)CommRcvBuff_;
        for (int i = 0; i < FN_Num; i++)
        {
          RCLCPP_INFO(rclcpp::get_logger("LeptrinoForceTorqueSensor"), "\tLimit[%d]: %f", i,
                      stGetLimit->fLimit[i]);
          conversion_factor_[i] = stGetLimit->fLimit[i] * 1e-4;
        }
        got_limit = true;
        break;
      }
    }
    wait_rate.sleep();
  }
  if (!got_limit)
  {
    RCLCPP_ERROR(rclcpp::get_logger("LeptrinoForceTorqueSensor"),
                 "Timeout while getting limit information");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
LeptrinoForceTorqueSensor::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Activate sensor
  SerialStart(rclcpp::get_logger("LeptrinoForceTorqueSensor"));

  // Skip calibration if the length is not defined
  if (calib_len_ <= 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("LeptrinoForceTorqueSensor"),
                "Calibration length is not defined, using no calibration");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Initialize the calibration offset
  rclcpp::Rate loop_rate(g_rate_);
  RCLCPP_INFO(rclcpp::get_logger("LeptrinoForceTorqueSensor"), "Calibration length is %d",
              calib_len_);
  int loop_counter = -kCalibIgnoreFirst; // Ignore first 100 samples
  while (rclcpp::ok() && loop_counter < calib_len_)
  {
    Comm_Rcv();
    if (Comm_CheckRcv() != 0)
    { // Receive data
      CommRcvBuff_[0] = 0;

      auto rt = Comm_GetRcvData(CommRcvBuff_);
      if (rt > 0)
      {
        auto stForce = (ST_R_DATA_GET_F *)CommRcvBuff_;
        if (loop_counter < 0)
        {
          // Ignore first 100 samples
          loop_counter++;
          continue;
        }
        for (int i = 0; i < FN_Num; i++)
        {
          calib_offset_[i] += stForce->ssForce[i] * conversion_factor_[i];
        }
        loop_counter++;
      }
    }
    else
    {
      loop_rate.sleep();
    }
  }
  for (int i = 0; i < FN_Num; i++)
  {
    calib_offset_[i] /= calib_len_;
    RCLCPP_INFO(rclcpp::get_logger("LeptrinoForceTorqueSensor"), "Calibration offset[%d]: %f", i,
                calib_offset_[i]);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
LeptrinoForceTorqueSensor::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop the sensor
  SerialStop(rclcpp::get_logger("LeptrinoForceTorqueSensor"));
  // Close the application
  App_Close(rclcpp::get_logger("LeptrinoForceTorqueSensor"));

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
LeptrinoForceTorqueSensor::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Ensure the stream is stopped and port is closed
  SerialStop(rclcpp::get_logger("LeptrinoForceTorqueSensor"));
  App_Close(rclcpp::get_logger("LeptrinoForceTorqueSensor"));
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
LeptrinoForceTorqueSensor::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Same as cleanup: ensure a clean shutdown to avoid device lingering in streaming mode
  SerialStop(rclcpp::get_logger("LeptrinoForceTorqueSensor"));
  App_Close(rclcpp::get_logger("LeptrinoForceTorqueSensor"));
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type LeptrinoForceTorqueSensor::read(const rclcpp::Time & /*time*/,
                                                                const rclcpp::Duration & /*period*/)
{
  // Read the data from the sensor
  Comm_Rcv();
  if (Comm_CheckRcv() != 0)
  { // Receive data
    memset(CommRcvBuff_, 0, sizeof(CommRcvBuff_));
    auto rt = Comm_GetRcvData(CommRcvBuff_);
    if (rt > 0)
    {
      auto stForce = (ST_R_DATA_GET_F *)CommRcvBuff_;
      auto &clk = *clock_;
      RCLCPP_DEBUG_THROTTLE(get_logger(), clk, 0.1, "%d,%d,%d,%d,%d,%d", stForce->ssForce[0],
                            stForce->ssForce[1], stForce->ssForce[2], stForce->ssForce[3],
                            stForce->ssForce[4], stForce->ssForce[5]);

      for (int i = 0; i < FN_Num; i++)
      {
        hw_sensor_states_[i] = stForce->ssForce[i] * conversion_factor_[i] - calib_offset_[i];
      }
    }
  }

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

void LeptrinoForceTorqueSensor::DrainRx(std::chrono::milliseconds duration)
{
  // Non-blocking drain of incoming bytes for a short period to clear stale frames
  auto start = std::chrono::steady_clock::now();
  while ((std::chrono::steady_clock::now() - start) < duration)
  {
    Comm_Rcv();
    if (Comm_CheckRcv() != 0)
    {
      memset(CommRcvBuff_, 0, sizeof(CommRcvBuff_));
      (void)Comm_GetRcvData(CommRcvBuff_);
    }
  }
}

} // namespace leptrino_force_torque

// ----------------------------------------------------------------------------
// Export plugin
// ----------------------------------------------------------------------------
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(leptrino_force_torque::LeptrinoForceTorqueSensor,
                       hardware_interface::SensorInterface)