#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include "cJSON.h"
#include "frame_struct.h"
#include "msa010.hpp"

#include "sipeed_tof_ms_a010_node.hh"

extern const uint8_t color_lut_jet[][3];

using namespace std::chrono_literals;

SipeedTOF_MSA010_Publisher::SipeedTOF_MSA010_Publisher()
: Node("sipeed_tof_ms_a010_ros_topic_publisher")
{

  this->declare_parameter("device", "/dev/ttyUSB0");
  rclcpp::Parameter device_param = this->get_parameter("device");
  s = device_param.as_string();
  std::cout << "use a device: " << s << std::endl;

  a010 = std::make_unique<msa010>(strdup(s.c_str()));

  std::string to_device(s.substr(5));

  std::stringstream ss;

  ss.str("");
  ss << to_device << "/depth";
  publisher_depth =
    this->create_publisher<sensor_msgs::msg::Image>("depth", 10);

  ss.str("");
  ss << to_device << "/cloud";
  publisher_pointcloud =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);

  timer_ = this->create_wall_timer(
    30ms, std::bind(&SipeedTOF_MSA010_Publisher::timer_callback, this));
}


void SipeedTOF_MSA010_Publisher::timer_callback()
{
  if (!a010->is_connected()) {
    RCLCPP_INFO(get_logger(), "Try connecting...");
    a010->keep_connect([]() {return rclcpp::ok();});
    if (!a010->is_connected()) {
      return;
    }
    RCLCPP_INFO(get_logger(), "Connect success.");

    a010 << "AT+DISP=1\r";
    do {
      a010 >> s;
    } while (s.size());

    a010 << "AT\r";
    a010 >> s;
    if (s.compare("OK\r\n")) {   // not this serial port
      return;
    }

    a010 << "AT+COEFF?\r";
    a010 >> s;
    if (s.compare("+COEFF=1\r\nOK\r\n")) {   // not this serial port
      return;
    }

    a010 >> s;
    if (!s.size()) {   // not this serial port
      return;
    }

    cJSON * cparms = cJSON_ParseWithLength((const char *)s.c_str(), s.length());
    uvf_parms[0] =
      ((float)((cJSON_GetObjectItem(cparms, "fx")->valueint) / 262144.0f));
    uvf_parms[1] =
      ((float)((cJSON_GetObjectItem(cparms, "fy")->valueint) / 262144.0f));
    uvf_parms[2] =
      ((float)((cJSON_GetObjectItem(cparms, "u0")->valueint) / 262144.0f));
    uvf_parms[3] =
      ((float)((cJSON_GetObjectItem(cparms, "v0")->valueint) / 262144.0f));
    std::cout << "fx: " << uvf_parms[0] << std::endl;
    std::cout << "fy: " << uvf_parms[1] << std::endl;
    std::cout << "u0: " << uvf_parms[2] << std::endl;
    std::cout << "v0: " << uvf_parms[3] << std::endl;

    a010 << "AT+UNIT=4\r";
    a010 >> s;
    if (s.compare("OK\r\n")) {   // not this serial port
      return;
    }

    a010 << "AT+DISP=3\r";
    // a010 >> s;
    // if (s.compare("OK\r\n")) {  // not this serial port
    //   continue;
    // }
  }
  a010 >> s;
  if (!s.size()) {return;}

  extern frame_t * handle_process(const std::string & s);
  f = handle_process(s);
  if (!f) {return;}
  count += 1;

  uint8_t rows, cols, * depth;
  rows = f->frame_head.resolution_rows;
  cols = f->frame_head.resolution_cols;
  depth = f->payload;
  cv::Mat md(rows, cols, CV_8UC1, depth);

  RCLCPP_INFO(get_logger(), "Publishing %8lu's frame.", count);
  std_msgs::msg::Header header;
  header.stamp = this->now();
  header.frame_id = "map";
  {
    sensor_msgs::msg::Image msg_depth =
      *cv_bridge::CvImage(header, "mono8", md).toImageMsg().get();
    publisher_depth->publish(msg_depth);
  }
  {
    sensor_msgs::msg::PointCloud2 pcmsg;
    pcmsg.header = header;
    pcmsg.height = rows;
    pcmsg.width = cols;
    pcmsg.is_bigendian = false;
    pcmsg.point_step = 16;
    pcmsg.row_step = pcmsg.point_step * rows;
    pcmsg.is_dense = false;
    pcmsg.fields.resize(pcmsg.point_step / 4);
    pcmsg.fields[0].name = "x";
    pcmsg.fields[0].offset = 0;
    pcmsg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcmsg.fields[0].count = 1;
    pcmsg.fields[1].name = "y";
    pcmsg.fields[1].offset = 4;
    pcmsg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcmsg.fields[1].count = 1;
    pcmsg.fields[2].name = "z";
    pcmsg.fields[2].offset = 8;
    pcmsg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcmsg.fields[2].count = 1;
    pcmsg.fields[3].name = "rgb";
    pcmsg.fields[3].offset = 12;
    pcmsg.fields[3].datatype = sensor_msgs::msg::PointField::UINT32;
    pcmsg.fields[3].count = 1;
    float fox = uvf_parms[0];
    float foy = uvf_parms[1];
    float u0 = uvf_parms[2];
    float v0 = uvf_parms[3];
    pcmsg.data.resize(
      (pcmsg.height) * (pcmsg.width) * (pcmsg.point_step),
      0x00);
    uint8_t * ptr = pcmsg.data.data();
    for (unsigned int j = 0; j < pcmsg.height; j++) {
      for (unsigned int i = 0; i < pcmsg.width; i++) {
        float cx = (((float)i) - u0) / fox;
        float cy = (((float)j) - v0) / foy;
        float dst = ((float)depth[j * (pcmsg.width) + i]) / 1000;
        // float x = dst * cx;
        // float y = dst * cy;
        // float z = dst;
        float x = dst * cx;
        float z = -dst * cy;
        float y = dst;
        *((float *)(ptr + 0)) = x;
        *((float *)(ptr + 4)) = y;
        *((float *)(ptr + 8)) = z;
        const uint8_t * color = color_lut_jet[depth[j * (pcmsg.width) + i]];
        uint32_t color_r = color[0];
        uint32_t color_g = color[1];
        uint32_t color_b = color[2];
        *((uint32_t *)(ptr + 12)) =
          (color_r << 16) | (color_g << 8) | (color_b << 0);
        ptr += pcmsg.point_step;
      }
    }
    publisher_pointcloud->publish(pcmsg);
  }
}

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SipeedTOF_MSA010_Publisher>());
  rclcpp::shutdown();

  return 0;
}
