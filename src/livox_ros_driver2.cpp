//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <iostream>
#include <chrono>
#include <vector>
#include <csignal>
#include <thread>

#include "include/livox_ros_driver2.h"
#include "include/ros_headers.h"
#include "driver_node.h"
#include "lddc.h"
#include "lds_lidar.h"
#include "livox_lidar_api.h"

using namespace livox_ros;

#ifdef BUILDING_ROS1
int main(int argc, char **argv) {
  /** Ros related */
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::init(argc, argv, "livox_lidar_publisher");

  // ros::NodeHandle livox_node;
  livox_ros::DriverNode livox_node;

  DRIVER_INFO(livox_node, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq  = 10.0; /* Hz */
  int output_type      = kOutputToRos;
  std::string frame_id = "livox_frame";
  bool lidar_bag = true;
  bool imu_bag   = false;

  livox_node.GetNode().getParam("xfer_format", xfer_format);
  livox_node.GetNode().getParam("multi_topic", multi_topic);
  livox_node.GetNode().getParam("data_src", data_src);
  livox_node.GetNode().getParam("publish_freq", publish_freq);
  livox_node.GetNode().getParam("output_data_type", output_type);
  livox_node.GetNode().getParam("frame_id", frame_id);
  livox_node.GetNode().getParam("enable_lidar_bag", lidar_bag);
  livox_node.GetNode().getParam("enable_imu_bag", imu_bag);

  printf("data source:%u.\n", data_src);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  } else {
    publish_freq = publish_freq;
  }

  livox_node.future_ = livox_node.exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  livox_node.lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type,
                        publish_freq, frame_id, lidar_bag, imu_bag);
  livox_node.lddc_ptr_->SetRosNode(&livox_node);

  if (data_src == kSourceRawLidar) {
    DRIVER_INFO(livox_node, "Data Source is raw lidar.");

    std::string user_config_path;
    livox_node.getParam("user_config_path", user_config_path);
    DRIVER_INFO(livox_node, "Config file : %s", user_config_path.c_str());

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    livox_node.lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(livox_node, "Init lds lidar successfully!");
    } else {
      DRIVER_ERROR(livox_node, "Init lds lidar failed!");
    }
  } else {
    DRIVER_ERROR(livox_node, "Invalid data src (%d), please check the launch file", data_src);
  }

  livox_node.pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, &livox_node);
  livox_node.imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, &livox_node);
  while (ros::ok()) { usleep(10000); }

  return 0;
}

#elif defined BUILDING_ROS2
namespace livox_ros
{
struct FwResult {
  bool success = false;
  uint8_t version[4] = {0};
};

DriverNode::DriverNode(const rclcpp::NodeOptions & node_options)
: Node("livox_driver_node", node_options)
{
  DRIVER_INFO(*this, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq = 10.0; /* Hz */
  int output_type = kOutputToRos;
  std::string frame_id;

  this->declare_parameter("xfer_format", xfer_format);
  this->declare_parameter("multi_topic", 0);
  this->declare_parameter("data_src", data_src);
  this->declare_parameter("publish_freq", 10.0);
  this->declare_parameter("output_data_type", output_type);
  this->declare_parameter("frame_id", "frame_default");
  this->declare_parameter("user_config_path", "path_default");
  this->declare_parameter("cmdline_input_bd_code", "000000000000001");
  this->declare_parameter("lvx_file_path", "/home/livox/livox_test.lvx");

  this->get_parameter("xfer_format", xfer_format);
  this->get_parameter("multi_topic", multi_topic);
  this->get_parameter("data_src", data_src);
  this->get_parameter("publish_freq", publish_freq);
  this->get_parameter("output_data_type", output_type);
  this->get_parameter("frame_id", frame_id);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  } else {
    publish_freq = publish_freq;
  }

  future_ = exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type, publish_freq, frame_id);
  lddc_ptr_->SetRosNode(this);

  if (data_src == kSourceRawLidar) {
    DRIVER_INFO(*this, "Data Source is raw lidar.");

    std::string user_config_path;
    this->get_parameter("user_config_path", user_config_path);
    DRIVER_INFO(*this, "Config file : %s", user_config_path.c_str());

    std::string cmdline_bd_code;
    this->get_parameter("cmdline_input_bd_code", cmdline_bd_code);

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    read_lidar->SetEnableImu(enable_imu_);
    lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(*this, "Init lds lidar success!");
    } else {
      DRIVER_ERROR(*this, "Init lds lidar fail!");
    }
  } else {
    DRIVER_ERROR(*this, "Invalid data src (%d), please check the launch file", data_src);
  }

  // 서비스 서버 생성
  reboot_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/livox/reboot", std::bind(&DriverNode::RebootCallback, this, std::placeholders::_1, std::placeholders::_2));
  firmware_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/livox/firmware_version", std::bind(&DriverNode::GetFirmwareCallback, this, std::placeholders::_1, std::placeholders::_2));

  pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, this);
  if (enable_imu_) {
    imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, this);
  }
}

}  // namespace livox_ros

// 재부팅 서비스 콜백
void DriverNode::RebootCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  DRIVER_INFO(*this, "Reboot service called");

  // lddc_ptr_를 통해 Lds 접근
  if (!lddc_ptr_ || !lddc_ptr_->lds_) {
    response->success = false;
    response->message = "LDS not initialized";
    DRIVER_ERROR(*this, "LDS not initialized");
    return;
  }

  LdsLidar *lds_lidar = static_cast<LdsLidar*>(lddc_ptr_->lds_);
  bool sent = false;
  int reboot_count = 0;

  DRIVER_INFO(*this, "Scanning for connected LiDARs (total count: %u)", lds_lidar->lidar_count_);

  for (uint32_t i = 0; i < lds_lidar->lidar_count_; ++i) {
    LidarDevice* lidar = &lds_lidar->lidars_[i];

    // 라이다 재부팅 요청
    if (lidar->handle != 0) {
      DRIVER_INFO(*this, "Found connected LiDAR at index %u, handle: %u, state: %u",
                  i, lidar->handle, lidar->connect_state);

      livox_status status = LivoxLidarRequestReboot(lidar->handle,
        [](livox_status status, uint32_t handle, LivoxLidarRebootResponse*, void* node_ptr) {
          auto* driver_node = static_cast<DriverNode*>(node_ptr);
          if (status == kLivoxLidarStatusSuccess) {
            DRIVER_INFO(*driver_node, "Reboot command sent successfully to handle %u", handle);
          } else {
            DRIVER_ERROR(*driver_node, "Failed to send reboot command to handle %u, status: %d",
                        handle, status);
          }
        }, this);

      if (status == kLivoxLidarStatusSuccess) {
        sent = true;
        reboot_count++;
        DRIVER_INFO(*this, "Reboot request sent for handle %u", lidar->handle);
      } else {
        DRIVER_ERROR(*this, "Failed to send reboot request for handle %u, status: %d",
                    lidar->handle, status);
      }
    }
  }

  if (sent) {
    char msg[128];
    snprintf(msg, sizeof(msg), "Reboot command sent to %d LiDAR(s). Device will restart in ~30-60s.", reboot_count);
    response->success = true;
    response->message = msg;
    DRIVER_INFO(*this, "%s", msg);
  } else {
    response->success = false;
    response->message = "No connected LiDAR found in sampling state";
    DRIVER_WARN(*this, "No connected LiDAR found in sampling state");
  }
}

void DriverNode::GetFirmwareCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  DRIVER_INFO(*this, "Firmware version service called");

  if (!lddc_ptr_ || !lddc_ptr_->lds_) {
    response->success = false;
    response->message = "LDS not initialized";
    return;
  }

  LdsLidar *lds_lidar = static_cast<LdsLidar*>(lddc_ptr_->lds_);
  uint32_t active_handle = UINT32_MAX;

  for (uint32_t i = 0; i < lds_lidar->lidar_count_; ++i) {
    LidarDevice* lidar = &lds_lidar->lidars_[i];
    if (lidar->connect_state == kConnectStateSampling) {
      active_handle = lidar->handle;
      break;
    }
  }

  if (active_handle == UINT32_MAX) {
    response->success = false;
    response->message = "No connected LiDAR found in sampling state";
    return;
  }

  auto promise = std::make_shared<std::promise<FwResult>>();
  auto future = promise->get_future();

  // 내부 정보 쿼리 실행
  livox_status query_status = QueryLivoxLidarInternalInfo(active_handle,
    [](livox_status status, uint32_t handle, LivoxLidarDiagInternalInfoResponse *res, void *client_data) {
      auto pr = static_cast<std::promise<FwResult>*>(client_data);
      FwResult result;
      result.success = false;

      if (status == kLivoxLidarStatusSuccess && res && res->ret_code == 0) {
        uint8_t* p_data = res->data;
        uint16_t param_num = res->param_num;
        uint32_t offset = 0;

        for (uint16_t i = 0; i < param_num; ++i) {
          // 구조체 포인터로 캐스팅하여 데이터 접근
          LivoxLidarKeyValueParam* kv = reinterpret_cast<LivoxLidarKeyValueParam*>(p_data + offset);

          // Firmware App Version 키(0x8002) 확인
          if (kv->key == kKeyVersionApp) {
            if (kv->length >= 4) {
              memcpy(result.version, kv->value, 4);
              result.success = true;
            }
            break; // 원하는 정보를 찾았으므로 루프 종료
          }
          // 다음 파라미터 위치로 오프셋 이동 (Key 2바이트 + Length 2바이트 + 실제 데이터 길이)
          offset += (sizeof(kv->key) + sizeof(kv->length) + kv->length);
        }
      }

      if (!result.success) {
        printf("[Livox] Failed to find version info in response. Status: %d, Handle: %u\n", status, handle);
      }
      pr->set_value(result);
    }, promise.get());

  if (query_status != kLivoxLidarStatusSuccess) {
    response->success = false;
    response->message = "Failed to send query request";
    return;
  }

  if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
    FwResult res = future.get();
    if (res.success) {
      char buf[128];
      // Livox 펌웨어는 보통 V1.V2.V3.V4 형식을 가짐
      snprintf(buf, sizeof(buf), "Firmware Version: %u.%u.%u.%u (Handle: %u)",
               res.version[0], res.version[1], res.version[2], res.version[3], active_handle);
      response->success = true;
      response->message = buf;
      DRIVER_INFO(*this, "%s", buf);
    } else {
      response->success = false;
      response->message = "Failed to parse firmware version from LiDAR response";
    }
  } else {
    response->success = false;
    response->message = "Timeout: LiDAR did not respond";
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::DriverNode)

#endif  // defined BUILDING_ROS2


void DriverNode::PointCloudDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributePointCloudData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

void DriverNode::ImuDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributeImuData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}
