// Copyright 2026 Pavlou Fire Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <map>
#include <memory>
#include <thread>
#include <vector>

#include "gscam/gscam.hpp"
#include "gtest/gtest.h"
#include "sensor_msgs/msg/camera_info.hpp"

// Foxy's GoogleTest uses the older name for parameterized test instantiation.
#ifndef INSTANTIATE_TEST_SUITE_P
#define INSTANTIATE_TEST_SUITE_P INSTANTIATE_TEST_CASE_P
#endif

using namespace std::chrono_literals;

class StreamTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
  }
  void TearDown() override {context_->shutdown("test finished");}

  rclcpp::NodeOptions options(const std::string & pipeline)
  {
    rclcpp::NodeOptions result;
    result.context(context_);
    result.parameter_overrides(
      {rclcpp::Parameter("gscam_config", pipeline),
        rclcpp::Parameter("sync_sink", false),
        rclcpp::Parameter("use_sensor_data_qos", true),
        rclcpp::Parameter("use_image_transport", false),
        rclcpp::Parameter("startup_timeout_sec", 0.3),
        rclcpp::Parameter("sample_timeout_sec", 0.3)});
    return result;
  }

  void expect_stopped(const rclcpp::NodeOptions & node_options)
  {
    auto node = std::make_shared<gscam::GSCam>(node_options);
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (context_->is_valid() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
    EXPECT_FALSE(context_->is_valid());
  }

  std::shared_ptr<rclcpp::Context> context_;
};

struct PublicationMode
{
  bool gst_timestamps;
  bool image_transport;
};

class PublicationTest : public StreamTest,
  public ::testing::WithParamInterface<PublicationMode> {};

TEST_P(PublicationTest, preserves_pixels_stride_and_matching_headers) {
  auto node_options = options(
    "videotestsrc is-live=true pattern=red ! "
    "capsfilter caps=video/x-raw,format=RGB,width=641,height=480,framerate=30/1");
  auto parameters = node_options.parameter_overrides();
  const auto mode = GetParam();
  for (auto & parameter : parameters) {
    if (parameter.get_name() == "use_image_transport") {
      parameter = rclcpp::Parameter("use_image_transport", mode.image_transport);
    }
  }
  parameters.emplace_back("use_gst_timestamps", mode.gst_timestamps);
  parameters.emplace_back("frame_id", "front_wide_angle_camera_frame");
  parameters.emplace_back(
    "camera.image_raw.enable_pub_plugins",
    std::vector<std::string>{"image_transport/raw"});
  node_options.parameter_overrides(parameters);
  auto camera = std::make_shared<gscam::GSCam>(node_options);
  rclcpp::NodeOptions subscriber_options;
  subscriber_options.context(context_);
  auto subscriber = std::make_shared<rclcpp::Node>("image_checker", subscriber_options);
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context_;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(camera);
  executor.add_node(subscriber);

  std::map<int64_t, std_msgs::msg::Header> image_headers;
  std::map<int64_t, std_msgs::msg::Header> info_headers;
  int64_t last_stamp = 0;
  const auto qos = rclcpp::SensorDataQoS().keep_last(5);
  auto image_sub = subscriber->create_subscription<sensor_msgs::msg::Image>(
    "camera/image_raw", qos, [&](sensor_msgs::msg::Image::ConstSharedPtr image) {
      EXPECT_EQ(image->width, 641u);
      EXPECT_EQ(image->height, 480u);
      EXPECT_EQ(image->encoding, "rgb8");
      EXPECT_EQ(image->step, 641u * 3);
      ASSERT_EQ(image->data.size(), image->step * image->height);
      // An odd width produces padded GStreamer rows. Check every published row.
      for (size_t row = 0; row < image->height; ++row) {
        const auto offset = row * image->step;
        EXPECT_EQ(image->data[offset], 255);
        EXPECT_EQ(image->data[offset + 1], 0);
        EXPECT_EQ(image->data[offset + 2], 0);
      }
      const auto stamp = rclcpp::Time(image->header.stamp).nanoseconds();
      EXPECT_GT(stamp, last_stamp);
      EXPECT_EQ(image->header.frame_id, "front_wide_angle_camera_frame");
      last_stamp = stamp;
      image_headers[stamp] = image->header;
    });
  auto info_sub = subscriber->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera/camera_info", qos, [&](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
      EXPECT_EQ(info->width, 641u);
      EXPECT_EQ(info->height, 480u);
      EXPECT_EQ(info->k[0], 0.0);    // No invented calibration.
      info_headers[rclcpp::Time(info->header.stamp).nanoseconds()] = info->header;
    });
  const auto deadline = std::chrono::steady_clock::now() + 4s;
  size_t matched = 0;
  while (context_->is_valid() && matched < 20 && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    matched = 0;
    for (const auto & entry : image_headers) {
      const auto info = info_headers.find(entry.first);
      if (info != info_headers.end()) {
        EXPECT_EQ(entry.second.frame_id, info->second.frame_id);
        EXPECT_EQ(entry.second.stamp, info->second.stamp);
        ++matched;
      }
    }
    std::this_thread::sleep_for(5ms);
  }
  EXPECT_GE(matched, 20u);
  const auto stopped_at = std::chrono::steady_clock::now();
  executor.remove_node(camera);
  camera.reset();
  EXPECT_LT(std::chrono::steady_clock::now() - stopped_at, 1s);
}

INSTANTIATE_TEST_SUITE_P(
  TimestampsAndTransports, PublicationTest,
  ::testing::Values(
    PublicationMode{false, false},
    PublicationMode{true, false},
    PublicationMode{false, true},
    PublicationMode{true, true}));

TEST_F(StreamTest, missing_decoder_rejects_partial_pipeline) {
  expect_stopped(options("videotestsrc ! m20_nonexistent_decoder ! videoconvert"));
}

TEST_F(StreamTest, pipeline_with_its_own_sink_is_rejected) {
  expect_stopped(options("videotestsrc ! fakesink"));
}

TEST_F(StreamTest, invalid_configuration_stops_executor) {
  auto node_options = options("videotestsrc");
  auto parameters = node_options.parameter_overrides();
  parameters.emplace_back("image_encoding", "unsupported");
  node_options.parameter_overrides(parameters);
  expect_stopped(node_options);
}

TEST_F(StreamTest, source_without_frames_does_not_hang) {
  expect_stopped(
    options(
      "appsrc is-live=true format=time ! "
      "capsfilter caps=video/x-raw,format=RGB,width=16,height=16,framerate=30/1"));
}
