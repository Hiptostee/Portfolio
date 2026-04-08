/**
 * @file main.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR LD06 products 
 * sold by Shenzhen LDROBOT Co., LTD    
 * @version 0.1
 * @date 2021-10-28
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ros2_api.h"
#include "ldlidar_driver.h"

static int bins = 501;

void  ToLaserscanMessagePublish(ldlidar::Points2D& src, double lidar_spin_freq, LaserScanSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& lidarpub);

uint64_t GetSystemTimeStamp(void);

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ldlidar_published"); // create a ROS2 Node
  std::string product_name;
	std::string topic_name;
	std::string port_name;
  int serial_port_baudrate;
  std::string server_ip;
  std::string server_port;
  bool enable_serial_or_network_communication;
  ldlidar::LDType type_name;
  LaserScanSetting setting;
	setting.frame_id = "base_laser";
  setting.laser_scan_dir = true;
  setting.enable_angle_crop_func = false;
  setting.angle_crop_min = 0.0;
  setting.angle_crop_max = 0.0;  
  // declare ros2 param
  node->declare_parameter<std::string>("product_name", product_name);
  node->declare_parameter<std::string>("topic_name", topic_name);
  node->declare_parameter<std::string>("frame_id", setting.frame_id);
  node->declare_parameter<bool>("enable_serial_or_network_communication", enable_serial_or_network_communication);
  node->declare_parameter<std::string>("port_name", port_name);
  node->declare_parameter<int>("port_baudrate", serial_port_baudrate);
  node->declare_parameter<std::string>("server_ip", server_ip);
  node->declare_parameter<std::string>("server_port", server_port);
  node->declare_parameter<bool>("laser_scan_dir", setting.laser_scan_dir);
  node->declare_parameter<bool>("enable_angle_crop_func", setting.enable_angle_crop_func);
  node->declare_parameter<double>("angle_crop_min", setting.angle_crop_min);
  node->declare_parameter<double>("angle_crop_max", setting.angle_crop_max);
  node->declare_parameter<int>("measure_point_freq", setting.measure_point_freq);
  node->declare_parameter<int>("bins", bins);

  // get ros2 param
  node->get_parameter("product_name", product_name);
  node->get_parameter("topic_name", topic_name);
  node->get_parameter("frame_id", setting.frame_id);
  node->get_parameter("enable_serial_or_network_communication", enable_serial_or_network_communication);
  node->get_parameter("port_name", port_name);
  node->get_parameter("port_baudrate", serial_port_baudrate);
  node->get_parameter("server_ip", server_ip);
  node->get_parameter("server_port", server_port);
  node->get_parameter("laser_scan_dir", setting.laser_scan_dir);
  node->get_parameter("enable_angle_crop_func", setting.enable_angle_crop_func);
  node->get_parameter("angle_crop_min", setting.angle_crop_min);
  node->get_parameter("angle_crop_max", setting.angle_crop_max);
  node->get_parameter("measure_point_freq", setting.measure_point_freq);
  node->get_parameter("bins", bins);


  ldlidar::LDLidarDriver* ldlidarnode = new ldlidar::LDLidarDriver();

  RCLCPP_INFO(node->get_logger(), "LDLiDAR SDK Pack Version is: %s", ldlidarnode->GetLidarSdkVersionNumber().c_str());
  RCLCPP_INFO(node->get_logger(), "<product_name>: %s", product_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<topic_name>: %s", topic_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<frame_id>: %s", setting.frame_id.c_str());
  RCLCPP_INFO(node->get_logger(), "<enable_serial_or_network_communication>: %s", 
    (enable_serial_or_network_communication?"Enable serial":"Enable network"));
  RCLCPP_INFO(node->get_logger(), "<port_name>: %s", port_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<port_baudrate>: %d", serial_port_baudrate);
  RCLCPP_INFO(node->get_logger(), "<server_ip>: %s", server_ip.c_str());
  RCLCPP_INFO(node->get_logger(), "<server_port>: %s", server_port.c_str());
  RCLCPP_INFO(node->get_logger(), "<laser_scan_dir>: %s", (setting.laser_scan_dir?"Counterclockwise":"Clockwise"));
  RCLCPP_INFO(node->get_logger(), "<enable_angle_crop_func>: %s", (setting.enable_angle_crop_func?"true":"false"));
  RCLCPP_INFO(node->get_logger(), "<angle_crop_min>: %f", setting.angle_crop_min);
  RCLCPP_INFO(node->get_logger(), "<angle_crop_max>: %f", setting.angle_crop_max);
  RCLCPP_INFO(node->get_logger(), "<measure_point_freq>: %d", setting.measure_point_freq);

  if (product_name == "LDLiDAR_LD06") {
    type_name = ldlidar::LDType::LD_06;
  } else if (product_name == "LDLiDAR_LD19") {
    type_name = ldlidar::LDType::LD_19;
  } else if (product_name == "LDLiDAR_STL06P") {
    type_name = ldlidar::LDType::STL_06P;
  } else if (product_name == "LDLiDAR_STL27L") {
    type_name = ldlidar::LDType::STL_27L;
  }else if (product_name == "LDLiDAR_STL26") {
    type_name = ldlidar::LDType::STL_26;
  } else {
    RCLCPP_ERROR(node->get_logger(), "Error, input <product_name> is illegal.");
    exit(EXIT_FAILURE);
  }

  ldlidarnode->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp)); //   注册时间戳获取函数

  ldlidarnode->EnableFilterAlgorithnmProcess(true);

  if (enable_serial_or_network_communication) {
    if (ldlidarnode->Start(type_name, port_name, serial_port_baudrate, ldlidar::COMM_SERIAL_MODE)) {
      RCLCPP_INFO(node->get_logger(), "ldlidar node start is success");
    } else {
      RCLCPP_ERROR(node->get_logger(), "ldlidar node start is fail");
      exit(EXIT_FAILURE);
    }
  } else {
    if (ldlidarnode->Start(type_name, server_ip.c_str(), server_port.c_str(), ldlidar::COMM_TCP_CLIENT_MODE)) {
      RCLCPP_INFO(node->get_logger(), "ldldiar node start is success");
    } else {
      RCLCPP_ERROR(node->get_logger(), "ldlidar node start is fail");
      exit(EXIT_FAILURE);
    }
  }

  if (ldlidarnode->WaitLidarCommConnect(500)) {
    RCLCPP_INFO(node->get_logger(), "ldlidar communication is normal.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "ldlidar communication is abnormal.");
    exit(EXIT_FAILURE);
  }

  // create ldlidar data topic and publisher
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher = 
      node->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 10);
  
  rclcpp::WallRate r(10); //10hz

  ldlidar::Points2D laser_scan_points;
  double lidar_spin_freq;
  bool is_get = false;
  while (rclcpp::ok()) {
    switch (ldlidarnode->GetLaserScanData(laser_scan_points, 1000)){
      case ldlidar::LidarStatus::NORMAL: 
        if (!is_get) {
          is_get = true;
          RCLCPP_INFO(node->get_logger(), "get ldlidar normal data and publish topic message.");
        }
        ldlidarnode->GetLidarSpinFreq(lidar_spin_freq);
        ToLaserscanMessagePublish(laser_scan_points, lidar_spin_freq, setting, node, publisher);
        break;
      case ldlidar::LidarStatus::ERROR:
        RCLCPP_ERROR(node->get_logger(), "ldlidar driver error.");
        break;
      case ldlidar::LidarStatus::DATA_TIME_OUT:
        RCLCPP_ERROR(node->get_logger(), "get ldlidar data is time out, please check your lidar device.");
        break;
      case ldlidar::LidarStatus::DATA_WAIT:
        break;
      default:
        break;
    }

    r.sleep();
  }

  ldlidarnode->Stop();

  delete ldlidarnode;
  ldlidarnode = nullptr;

  RCLCPP_INFO(node->get_logger(), "ldlidar_published is end");
  rclcpp::shutdown();

  return 0;
}

void ToLaserscanMessagePublish(ldlidar::Points2D& src,
                              double lidar_spin_freq,
                              LaserScanSetting& setting,
                              rclcpp::Node::SharedPtr& node,
                              rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& lidarpub)
{
  float angle_min, angle_max, range_min, range_max, angle_increment;
  double scan_time;
  rclcpp::Time start_scan_time;
  static rclcpp::Time end_scan_time;
  static bool first_scan = true;

  start_scan_time = node->now();
  scan_time = (start_scan_time.seconds() - end_scan_time.seconds());

  if (first_scan) {
    first_scan = false;
    end_scan_time = start_scan_time;
    return;
  }

  // Fixed scan geometry
  angle_min = 0.0f;
  angle_max = static_cast<float>(2.0 * M_PI);
  range_min = 0.02f;
  range_max = 30.0f;

  const int beam_size_in = static_cast<int>(src.size());
  int beam_size_out = bins;               // force fixed output length
  if (beam_size_out < 2) beam_size_out = 2;       // prevent divide-by-zero
  // (If you ever want "dynamic mode", you could do: beam_size_out = (bins > 0 ? bins : beam_size_in))

  // Compute increment based on fixed output size
  angle_increment = (angle_max - angle_min) / static_cast<float>(beam_size_out - 1);

  if (lidar_spin_freq > 0) {
    sensor_msgs::msg::LaserScan output;
    output.header.stamp = start_scan_time;
    output.header.frame_id = setting.frame_id;
    output.angle_min = angle_min;
    output.angle_max = angle_max;
    output.range_min = range_min;
    output.range_max = range_max;
    output.angle_increment = angle_increment;

    output.time_increment =
        (beam_size_out <= 1) ? 0.0f : static_cast<float>(scan_time / static_cast<double>(beam_size_out - 1));
    output.scan_time = scan_time;

    // Fill fixed-size arrays
    output.ranges.assign(beam_size_out, std::numeric_limits<float>::quiet_NaN());
    output.intensities.assign(beam_size_out, std::numeric_limits<float>::quiet_NaN());

    for (const auto& point : src) {
      float range = point.distance / 1000.f;   // mm -> m
      float intensity = point.intensity;
      float dir_angle = point.angle;

      if ((point.distance == 0) && (point.intensity == 0)) {
        range = std::numeric_limits<float>::quiet_NaN();
        intensity = std::numeric_limits<float>::quiet_NaN();
      }

      if (setting.enable_angle_crop_func) {
        if ((dir_angle >= setting.angle_crop_min) && (dir_angle <= setting.angle_crop_max)) {
          range = std::numeric_limits<float>::quiet_NaN();
          intensity = std::numeric_limits<float>::quiet_NaN();
        }
      }

      const float angle = ANGLE_TO_RADIAN(dir_angle);

      // Bin into fixed-size scan
      int index = static_cast<int>(std::lround((angle - angle_min) / angle_increment));
      if (index < 0 || index >= beam_size_out) {
        continue;
      }

      if (setting.laser_scan_dir) {
        const int idx = beam_size_out - index - 1;
        if (std::isnan(output.ranges[idx]) || range < output.ranges[idx]) {
          output.ranges[idx] = range;
        }
        output.intensities[idx] = intensity;
      } else {
        if (std::isnan(output.ranges[index]) || range < output.ranges[index]) {
          output.ranges[index] = range;
        }
        output.intensities[index] = intensity;
      }
    }

    lidarpub->publish(output);
    end_scan_time = start_scan_time;
  }
}

uint64_t GetSystemTimeStamp(void) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp = 
    std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return ((uint64_t)tmp.count());
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/