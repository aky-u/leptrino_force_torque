#include "leptrino/system.hpp"

namespace leptrino_force_torque_sensor
{
  LeptrinoForceTorqueSensor::~LeptrinoForceTorqueSensor()
  {
    // If the controller manager is shutdown via Ctrl + C
    on_cleanup(rclcpp_lifecycle::State());
  }

  hardware_interface::CallbackReturn LeptrinoForceTorqueSensor::on_init(const hardware_interface::HardwareInfo &info)
  {
    // Initialize the sensor interface
    if (SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Store the information about the sensor
    g_com_port_ = info.hardware_parameters.at("com_port");
    g_rate_ = std::stoi(info.hardware_parameters.at("rate"));
    g_com_ok_ = 0;

    // Initialize the conversion factors
    conversion_factor_[0] = std::stod(info.hardware_parameters.at("conversion_factor_x"));
    conversion_factor_[1] = std::stod(info.hardware_parameters.at("conversion_factor_y"));
    conversion_factor_[2] = std::stod(info.hardware_parameters.at("conversion_factor_z"));
    conversion_factor_[3] = std::stod(info.hardware_parameters.at("conversion_factor_rx"));
    conversion_factor_[4] = std::stod(info.hardware_parameters.at("conversion_factor_ry"));
    conversion_factor_[5] = std::stod(info.hardware_parameters.at("conversion_factor_rz"));

    // // Initialize the hw state
    // for (size_t i = 0; i < state_interfaces_.size(); ++i)
    // {
    //   state_interfaces_[i].set_value(0.0);
    // }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ----------------------------------------------------------------------------
  // Private functions
  // ----------------------------------------------------------------------------
  void LeptrinoForceTorqueSensor::App_Init()
  {
    int rt;

    // Initialize the Comm port
    g_com_ok_ = NG;
    rt = Comm_Open(g_com_port_.c_str());
    if (rt == OK)
    {
      Comm_Setup(460800, PAR_NON, BIT_LEN_8, 0, 0, CHR_ETX);
      g_com_ok_ = OK;
    }
  }

  void LeptrinoForceTorqueSensor::App_Close(rclcpp::Logger logger)
  {
    RCLCPP_DEBUG(logger, "Application close\n");

    if (g_com_ok_ == OK)
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

    return OK;
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

} // namespace leptrino_force_torque_sensor