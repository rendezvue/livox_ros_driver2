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
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

using namespace livox_ros;
uint32_t front_lidar_handle = 71739584; // Front LiDAR handle

ros::Publisher diagnostic_pub;
std::set<uint32_t> window_issue_codes = {
  0x01040002, // Window dirty
  0x01040003, // Window blocked (assumed)
  0x01040004  // Window scratched (assumed)
};
// Parse HMS codes (copied from your original code)
bool parse_hms_codes(const std::string& info, std::vector<uint32_t>& hms_codes) {
    hms_codes.clear();
    if (info.empty()) {
        // DRIVER_WARN("parse_hms_codes: Input string is empty");
        return false;
    }
    std::string search_key = "\"hms_code\": [";
    size_t start_pos = info.find(search_key);
    if (start_pos == std::string::npos) {
        // DRIVER_WARN("parse_hms_codes: 'hms_code' key not found in info: %s", info.c_str());
        return false;
    }
    start_pos += search_key.length();
    size_t end_pos = info.find(']', start_pos);
    if (end_pos == std::string::npos) {
        // DRIVER_WARN("parse_hms_codes: Closing bracket not found in info: %s", info.c_str());
        return false;
    }
    std::string array_str = info.substr(start_pos, end_pos - start_pos);
    std::stringstream ss(array_str);
    std::string token;
    while (std::getline(ss, token, ',')) {
        token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
        if (!token.empty()) {
            try {
                uint32_t value = std::stoul(token);
                hms_codes.push_back(value);
            } catch (const std::exception& e) {
                // DRIVER_WARN("parse_hms_codes: Failed to parse HMS code value '%s': %s", token.c_str(), e.what());
            }
        }
    }
    if (hms_codes.empty()) {
        // DRIVER_WARN("parse_hms_codes: No valid HMS codes parsed from array: %s", array_str.c_str());
        return false;
    }
    return true;
}

// Status info callback for HMS codes
void StatusInfoCallback(uint32_t handle, uint8_t dev_type, const char* info, void* client_data) {
    // DRIVER_DEBUG("StatusInfoCallback called for handle %u, dev_type %u", handle, dev_type);

    if (info == nullptr) {
        // DRIVER_ERROR("StatusInfoCallback: info is nullptr for handle %u", handle);
        diagnostic_msgs::DiagnosticStatus diag_status;
        diag_status.name = (handle == front_lidar_handle) ? "Front Livox LiDAR" : "Rear Livox LiDAR";
        diag_status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
        diag_status.message = "Received null info string.";
        diagnostic_msgs::KeyValue kv;
        kv.key = "HMS Codes";
        kv.value = "None";
        diag_status.values.push_back(kv);
        diagnostic_pub.publish(diag_status);
        return;
    }

    std::vector<uint32_t> hms_codes;
    bool hms_code_found = parse_hms_codes(std::string(info), hms_codes);
    bool has_window_issue = false;
    std::string hms_str;
    std::vector<uint32_t> detected_issues;

    if (hms_code_found) {
        for (size_t i = 0; i < hms_codes.size(); ++i) {
            uint32_t code = hms_codes[i];
            char hex_str[12];
            snprintf(hex_str, sizeof(hex_str), "0x%08X", code);
            if (window_issue_codes.count(code) > 0) {
                has_window_issue = true;
                detected_issues.push_back(code);
            }
            hms_str += hex_str + std::string(" ");
        }
        // DRIVER_DEBUG("HMS codes string for handle %u: %s", handle, hms_str.c_str());
    }

    diagnostic_msgs::DiagnosticStatus diag_status;
    diag_status.name = (handle == front_lidar_handle) ? "Front Livox LiDAR" : "Rear Livox LiDAR";
    if (has_window_issue) {
        diag_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
        diag_status.message = "Window is dirty.";
    } else {
        diag_status.level = diagnostic_msgs::DiagnosticStatus::OK;
        diag_status.message = hms_code_found ? "Window is clean." : "No HMS codes received.";
    }
    diagnostic_msgs::KeyValue kv;
    kv.key = "HMS Codes";
    kv.value = hms_str.empty() ? "None" : hms_str;
    diag_status.values.push_back(kv);
    diagnostic_pub.publish(diag_status);
}

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
  diagnostic_pub = livox_node.GetNode().advertise<diagnostic_msgs::DiagnosticStatus>("/diagnostics", 1);

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
      SetLivoxLidarInfoCallback(StatusInfoCallback, nullptr);
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
    lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(*this, "Init lds lidar success!");
    } else {
      DRIVER_ERROR(*this, "Init lds lidar fail!");
    }
  } else {
    DRIVER_ERROR(*this, "Invalid data src (%d), please check the launch file", data_src);
  }

  pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, this);
  imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, this);
}

}  // namespace livox_ros

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





















