/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Imai Laboratory, Keio University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Imai Laboratory, nor the name of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mahisorn Wongphati
 * Notice: Modified & copied from Leptrino CD example source code
 * Notice: Modified for ROS2 by Vineet
 */

// =============================================================================
//  CFS_Sample 本体部
//
//          Filename: main.c
//
// =============================================================================
//    Ver 1.0.0   2022/06/01
// =============================================================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <leptrino/pComResInternal.h>
#include <leptrino/pCommon.h>
#include <leptrino/rs_comm.h>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <atomic>
#include <cmath>
#include <numeric>
#include <std_srvs/srv/trigger.hpp>
#include <thread>
#include <vector>

// =============================================================================
//  マクロ定義 Defining macros
// =============================================================================
#define PRG_VER "Ver 1.0.0"

// =============================================================================
//  構造体定義 Defining structs
// =============================================================================
typedef struct ST_SystemInfo
{
  int com_ok;
} SystemInfo;

// =============================================================================
//  プロトタイプ宣言 Declaring prototypes
// =============================================================================
void App_Init(void);
void App_Close(rclcpp::Logger logger);
ULONG SendData(UCHAR *pucInput, USHORT usSize);
void GetProductInfo(rclcpp::Logger logger);
void GetLimit(rclcpp::Logger logger);
void SerialStart(rclcpp::Logger logger);
void SerialStop(rclcpp::Logger logger);

// =============================================================================
//  モジュール変数定義 Defining module variables
// =============================================================================
SystemInfo gSys;
UCHAR CommRcvBuff[256];
UCHAR CommSendBuff[1024];
UCHAR SendBuff[512];
double conversion_factor[FN_Num];

std::string g_com_port;
int g_rate;

#define TEST_TIME 0

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("leptrino");
  node->declare_parameter("com_port", std::string("/dev/ttyACM0"));
  node->declare_parameter("rate", 1200);
  node->declare_parameter("calib_len", 100);
  node->declare_parameter("calib_start", 100);
  node->declare_parameter("auto_calibrate", true);
  node->declare_parameter("calib_std_threshold", 0.05);
  node->declare_parameter("persist_offsets", false);

  node->get_parameter("com_port", g_com_port);
  node->get_parameter("rate", g_rate);
  int calib_len = 100;
  int calib_start = 100;
  bool auto_calibrate = true;
  double calib_std_threshold = 0.05;
  bool persist_offsets = false;
  node->get_parameter("calib_len", calib_len);
  node->get_parameter("calib_start", calib_start);
  node->get_parameter("auto_calibrate", auto_calibrate);
  node->get_parameter("calib_std_threshold", calib_std_threshold);
  node->get_parameter("persist_offsets", persist_offsets);
  rclcpp::Rate rate(g_rate);

  std::string frame_id = "leptrino";
  node->get_parameter("frame_id", frame_id);

  int rt = 0;
  // ST_RES_HEAD *stCmdHead;
  ST_R_DATA_GET_F *stForce;
  ST_R_GET_INF *stGetInfo;
  ST_R_LEP_GET_LIMIT *stGetLimit;

  App_Init();

  if (gSys.com_ok == COM_NG)
  {
    RCLCPP_ERROR(node->get_logger(), "%s open failed", g_com_port.c_str());
    return 1;
  }

  // 製品情報取得
  GetProductInfo(node->get_logger());
  while (rclcpp::ok())
  {
    Comm_Rcv();
    if (Comm_CheckRcv() != 0)
    { // 受信データ有
      CommRcvBuff[0] = 0;

      rt = Comm_GetRcvData(CommRcvBuff);
      if (rt > 0)
      {
        stGetInfo = (ST_R_GET_INF *)CommRcvBuff;
        stGetInfo->scFVer[F_VER_SIZE] = 0;
        RCLCPP_INFO(node->get_logger(), "Version: %s", stGetInfo->scFVer);
        stGetInfo->scSerial[SERIAL_SIZE] = 0;
        RCLCPP_INFO(node->get_logger(), "SerialNo: %s", stGetInfo->scSerial);
        stGetInfo->scPName[P_NAME_SIZE] = 0;
        RCLCPP_INFO(node->get_logger(), "Type: %s", stGetInfo->scPName);
        break;
      }
    }
    else
    {
      rate.sleep();
    }
  }

  GetLimit(node->get_logger());
  while (rclcpp::ok())
  {
    Comm_Rcv();
    if (Comm_CheckRcv() != 0)
    { // 受信データ有
      CommRcvBuff[0] = 0;

      rt = Comm_GetRcvData(CommRcvBuff);
      if (rt > 0)
      {
        stGetLimit = (ST_R_LEP_GET_LIMIT *)CommRcvBuff;
        for (int i = 0; i < FN_Num; i++)
        {
          RCLCPP_INFO(node->get_logger(), "\tLimit[%d]: %f", i, stGetLimit->fLimit[i]);
          conversion_factor[i] = stGetLimit->fLimit[i] * 1e-4;
        }
        break;
      }
    }
    else
    {
      rate.sleep();
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_torque_pub =
      node->create_publisher<geometry_msgs::msg::WrenchStamped>("force_torque", 1);

  usleep(10000);

  // 連続送信開始
  SerialStart(node->get_logger());

#if TEST_TIME
  double dt_sum = 0;
  int dt_count = 0;
  rclcpp::Time start_time;
#endif

  int loop_counter = 0;
  auto msg_offset = geometry_msgs::msg::WrenchStamped();
  // ensure zero offsets
  msg_offset.wrench.force.x = 0.0;
  msg_offset.wrench.force.y = 0.0;
  msg_offset.wrench.force.z = 0.0;
  msg_offset.wrench.torque.x = 0.0;
  msg_offset.wrench.torque.y = 0.0;
  msg_offset.wrench.torque.z = 0.0;

  // Calibration state
  std::atomic_bool calibrated(false);
  std::atomic_bool calib_in_progress(false);
  int calib_samples_collected = 0;
  std::vector<double> sums(6, 0.0);
  std::vector<double> sums_sq(6, 0.0);

  // Service to trigger recalibration on demand
  auto recal_srv = node->create_service<std_srvs::srv::Trigger>(
      "recalibrate",
      [&](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
          std::shared_ptr<std_srvs::srv::Trigger::Response> res)
      {
        if (calib_in_progress.load())
        {
          res->success = false;
          res->message = "Calibration already in progress";
          return;
        }
        // reset accumulators
        calib_in_progress.store(true);
        calibrated.store(false);
        calib_samples_collected = 0;
        std::fill(sums.begin(), sums.end(), 0.0);
        std::fill(sums_sq.begin(), sums_sq.end(), 0.0);
        res->success = true;
        res->message = "Calibration started";
        RCLCPP_INFO(node->get_logger(), "Manual recalibration requested");
      });

  // spin node in a background thread so service/callbacks work while main loop runs
  std::thread spin_thread([&]() { rclcpp::spin(node); });

  while (rclcpp::ok())
  {
    Comm_Rcv();
    if (Comm_CheckRcv() != 0)
    { // 受信データ有

#if TEST_TIME
      dt_count++;
      dt_sum += (node->now() - start_time).toSec();
      if (dt_sum >= 1.0)
      {
        ROS_INFO("Time test: read %d in %6.3f sec: %6.3f kHz", dt_count, dt_sum,
                 (dt_count / dt_sum) * 0.001);
        dt_count = 0;
        dt_sum = 0.0;
      }
      start_time = node->now();
#endif

      memset(CommRcvBuff, 0, sizeof(CommRcvBuff));
      rt = Comm_GetRcvData(CommRcvBuff);
      if (rt > 0)
      {
        stForce = (ST_R_DATA_GET_F *)CommRcvBuff;
        auto &clk = *node->get_clock();
        RCLCPP_DEBUG_THROTTLE(node->get_logger(), clk, 0.1, "%d,%d,%d,%d,%d,%d",
                              stForce->ssForce[0], stForce->ssForce[1], stForce->ssForce[2],
                              stForce->ssForce[3], stForce->ssForce[4], stForce->ssForce[5]);

        auto msg = geometry_msgs::msg::WrenchStamped();
        msg.header.stamp = node->now();
        msg.header.frame_id = frame_id;
        msg.wrench.force.x = stForce->ssForce[0] * conversion_factor[0];
        msg.wrench.force.y = stForce->ssForce[1] * conversion_factor[1];
        msg.wrench.force.z = stForce->ssForce[2] * conversion_factor[2];
        msg.wrench.torque.x = stForce->ssForce[3] * conversion_factor[3];
        msg.wrench.torque.y = stForce->ssForce[4] * conversion_factor[4];
        msg.wrench.torque.z = stForce->ssForce[5] * conversion_factor[5];

        // Calibration: automatic at startup or manual via service
        if (!calibrated.load())
        {
          if (auto_calibrate && !calib_in_progress.load() && loop_counter >= calib_start)
          {
            // start automatic calibration
            calib_in_progress.store(true);
            calib_samples_collected = 0;
            std::fill(sums.begin(), sums.end(), 0.0);
            std::fill(sums_sq.begin(), sums_sq.end(), 0.0);
            RCLCPP_INFO(node->get_logger(), "Automatic calibration started");
          }

          if (calib_in_progress.load())
          {
            // accumulate
            double vals[6] = {msg.wrench.force.x,  msg.wrench.force.y,  msg.wrench.force.z,
                              msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z};
            for (int i = 0; i < 6; ++i)
            {
              sums[i] += vals[i];
              sums_sq[i] += vals[i] * vals[i];
            }
            calib_samples_collected++;

            if (calib_samples_collected >= calib_len)
            {
              // compute mean and stddev
              bool ok = true;
              double means[6];
              double stddev[6];
              for (int i = 0; i < 6; ++i)
              {
                means[i] = sums[i] / static_cast<double>(calib_samples_collected);
                double var = (sums_sq[i] / static_cast<double>(calib_samples_collected)) -
                             (means[i] * means[i]);
                stddev[i] = (var > 0.0) ? std::sqrt(var) : 0.0;
                if (std::abs(stddev[i]) > calib_std_threshold)
                {
                  ok = false;
                }
              }

              if (ok)
              {
                msg_offset.wrench.force.x = means[0];
                msg_offset.wrench.force.y = means[1];
                msg_offset.wrench.force.z = means[2];
                msg_offset.wrench.torque.x = means[3];
                msg_offset.wrench.torque.y = means[4];
                msg_offset.wrench.torque.z = means[5];
                calibrated.store(true);
                calib_in_progress.store(false);
                RCLCPP_INFO(node->get_logger(),
                            "Calibration done. Offsets: fx=%f fy=%f fz=%f tx=%f ty=%f tz=%f",
                            msg_offset.wrench.force.x, msg_offset.wrench.force.y,
                            msg_offset.wrench.force.z, msg_offset.wrench.torque.x,
                            msg_offset.wrench.torque.y, msg_offset.wrench.torque.z);

                if (persist_offsets)
                {
                  node->set_parameter(
                      rclcpp::Parameter("offset_force_x", msg_offset.wrench.force.x));
                  node->set_parameter(
                      rclcpp::Parameter("offset_force_y", msg_offset.wrench.force.y));
                  node->set_parameter(
                      rclcpp::Parameter("offset_force_z", msg_offset.wrench.force.z));
                  node->set_parameter(
                      rclcpp::Parameter("offset_torque_x", msg_offset.wrench.torque.x));
                  node->set_parameter(
                      rclcpp::Parameter("offset_torque_y", msg_offset.wrench.torque.y));
                  node->set_parameter(
                      rclcpp::Parameter("offset_torque_z", msg_offset.wrench.torque.z));
                }
              }
              else
              {
                RCLCPP_WARN(node->get_logger(),
                            "Calibration failed: stddev too large on one or more axes. stddevs: %f "
                            "%f %f %f %f %f",
                            stddev[0], stddev[1], stddev[2], stddev[3], stddev[4], stddev[5]);
                // reset to allow retry (manual or automatic)
                calib_in_progress.store(false);
                calib_samples_collected = 0;
                std::fill(sums.begin(), sums.end(), 0.0);
                std::fill(sums_sq.begin(), sums_sq.end(), 0.0);
              }
            }
          }
        }
        // if calibrated, subtract and publish
        if (calibrated.load())
        {
          msg.wrench.force.x -= msg_offset.wrench.force.x;
          msg.wrench.force.y -= msg_offset.wrench.force.y;
          msg.wrench.force.z -= msg_offset.wrench.force.z;
          msg.wrench.torque.x -= msg_offset.wrench.torque.x;
          msg.wrench.torque.y -= msg_offset.wrench.torque.y;
          msg.wrench.torque.z -= msg_offset.wrench.torque.z;

          force_torque_pub->publish(msg);
        }
        loop_counter++;
      }
    }
    else
    {
      rate.sleep();
    }

    // rclcpp::spin(node);
  } // while

  SerialStop(node->get_logger());
  App_Close(node->get_logger());
  return 0;
}

// ----------------------------------------------------------------------------------
//  アプリケーション初期化
// ----------------------------------------------------------------------------------
//  引数  : none
//  戻り値  : none
// ----------------------------------------------------------------------------------
void App_Init(void)
{
  int rt;

  // Commポート初期化
  gSys.com_ok = COM_NG;
  rt = Comm_Open(g_com_port.c_str());
  if (rt == COM_OK)
  {
    Comm_Setup(460800, PAR_NON, BIT_LEN_8, 0, 0, CHR_ETX);
    gSys.com_ok = COM_OK;
  }
}

// ----------------------------------------------------------------------------------
//  アプリケーション終了処理
// ----------------------------------------------------------------------------------
//  引数  : none
//  戻り値  : none
// ----------------------------------------------------------------------------------
void App_Close(rclcpp::Logger logger)
{
  RCLCPP_DEBUG(logger, "Application close\n");

  if (gSys.com_ok == COM_OK)
  {
    Comm_Close();
  }
}

/*********************************************************************************
 * Function Name  : HST_SendResp
 * Description    : データを整形して送信する
 * Input          : pucInput 送信データ
 *                : 送信データサイズ
 * Output         :
 * Return         :
 *********************************************************************************/
ULONG SendData(UCHAR *pucInput, USHORT usSize)
{
  USHORT usCnt;
  UCHAR ucWork;
  UCHAR ucBCC = 0;
  UCHAR *pucWrite = &CommSendBuff[0];
  USHORT usRealSize;

  // データ整形
  *pucWrite = CHR_DLE; // DLE
  pucWrite++;
  *pucWrite = CHR_STX; // STX
  pucWrite++;
  usRealSize = 2;

  for (usCnt = 0; usCnt < usSize; usCnt++)
  {
    ucWork = pucInput[usCnt];
    if (ucWork == CHR_DLE)
    {                      // データが0x10ならば0x10を付加
      *pucWrite = CHR_DLE; // DLE付加
      pucWrite++;          // 書き込み先
      usRealSize++;        // 実サイズ
      // BCCは計算しない!
    }
    *pucWrite = ucWork; // データ
    ucBCC ^= ucWork;    // BCC
    pucWrite++;         // 書き込み先
    usRealSize++;       // 実サイズ
  }

  *pucWrite = CHR_DLE; // DLE
  pucWrite++;
  *pucWrite = CHR_ETX; // ETX
  ucBCC ^= CHR_ETX;    // BCC計算
  pucWrite++;
  *pucWrite = ucBCC; // BCC付加
  usRealSize += 3;

  Comm_SendData(&CommSendBuff[0], usRealSize);

  return COM_OK;
}

void GetProductInfo(rclcpp::Logger logger)
{
  USHORT len;

  RCLCPP_INFO(logger, "Get sensor information");
  len = 0x04;                // データ長
  SendBuff[0] = len;         // レングス
  SendBuff[1] = 0xFF;        // センサNo.
  SendBuff[2] = CMD_GET_INF; // コマンド種別
  SendBuff[3] = 0;           // 予備

  SendData(SendBuff, len);
}

void GetLimit(rclcpp::Logger logger)
{
  USHORT len;

  RCLCPP_INFO(logger, "Get sensor limit");
  len = 0x04;
  SendBuff[0] = len;           // レングス length
  SendBuff[1] = 0xFF;          // センサNo. Sensor no.
  SendBuff[2] = CMD_GET_LIMIT; // コマンド種別 Command type
  SendBuff[3] = 0;             // 予備 reserve

  SendData(SendBuff, len);
}

void SerialStart(rclcpp::Logger logger)
{
  USHORT len;

  RCLCPP_INFO(logger, "Start sensor");
  len = 0x04;                   // データ長
  SendBuff[0] = len;            // レングス
  SendBuff[1] = 0xFF;           // センサNo.
  SendBuff[2] = CMD_DATA_START; // コマンド種別
  SendBuff[3] = 0;              // 予備

  SendData(SendBuff, len);
}

void SerialStop(rclcpp::Logger logger)
{
  USHORT len;

  RCLCPP_INFO(logger, "Stop sensor\n");
  len = 0x04;                  // データ長
  SendBuff[0] = len;           // レングス
  SendBuff[1] = 0xFF;          // センサNo.
  SendBuff[2] = CMD_DATA_STOP; // コマンド種別
  SendBuff[3] = 0;             // 予備

  SendData(SendBuff, len);
}
