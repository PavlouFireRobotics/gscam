// Copyright 2022 Jonathan Bohren, Clyde McQueen
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

#include <stdlib.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <cmath>
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>

extern "C" {
#include "gst/gst.h"
#include "gst/app/gstappsink.h"
#include "gst/video/video.h"
}

#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "gscam/gscam.hpp"
#include "gscam/appsink.hpp"

namespace gscam
{

GSCam::GSCam(const rclcpp::NodeOptions & options)
: rclcpp::Node("gscam_publisher", options),
  gsconfig_(""),
  pipeline_(NULL),
  sink_(NULL),
  camera_info_manager_(this, "camera", ""),
  stop_signal_(false)
{
  pipeline_thread_ = std::thread(
    [this]()
    {
      run();
    });
}

GSCam::~GSCam()
{
  stop_signal_ = true;
  if (pipeline_thread_.joinable()) {
    pipeline_thread_.join();
  }
  cleanup_stream();
}

bool GSCam::configure()
{
  // Get gstreamer configuration
  // (either from environment variable or ROS param)
  bool gsconfig_rosparam_defined = false;
  char * gsconfig_env = NULL;

  const auto gsconfig_rosparam = declare_parameter("gscam_config", "");
  gsconfig_rosparam_defined = !gsconfig_rosparam.empty();
  gsconfig_env = getenv("GSCAM_CONFIG");

  if (!gsconfig_env && !gsconfig_rosparam_defined) {
    RCLCPP_FATAL(
      get_logger(),
      "Problem getting GSCAM_CONFIG environment variable and "
      "'gscam_config' rosparam is not set. This is needed to set up a gstreamer pipeline.");
    return false;
  } else if (gsconfig_env && gsconfig_rosparam_defined) {
    RCLCPP_FATAL(
      get_logger(),
      "Both GSCAM_CONFIG environment variable and 'gscam_config' rosparam are set. "
      "Please only define one.");
    return false;
  } else if (gsconfig_env) {
    gsconfig_ = gsconfig_env;
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Using gstreamer config from env: \"" << gsconfig_env << "\"");
  } else if (gsconfig_rosparam_defined) {
    gsconfig_ = gsconfig_rosparam;
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Using gstreamer config from rosparam: \"" << gsconfig_rosparam << "\"");
  }

  // Get additional gscam configuration
  sync_sink_ = declare_parameter("sync_sink", true);
  preroll_ = declare_parameter("preroll", false);
  use_gst_timestamps_ = declare_parameter("use_gst_timestamps", false);

  sample_timeout_sec_ = declare_parameter("sample_timeout_sec", 5.0);
  if (!std::isfinite(sample_timeout_sec_) || sample_timeout_sec_ <= 0.0 ||
    sample_timeout_sec_ > 60.0)
  {
    RCLCPP_FATAL(get_logger(), "sample_timeout_sec must be in (0, 60].");
    return false;
  }

  startup_timeout_sec_ = declare_parameter("startup_timeout_sec", 5.0);
  if (!std::isfinite(startup_timeout_sec_) || startup_timeout_sec_ <= 0.0 ||
    startup_timeout_sec_ > 60.0)
  {
    RCLCPP_FATAL(get_logger(), "startup_timeout_sec must be in (0, 60].");
    return false;
  }

  reopen_on_eof_ = declare_parameter("reopen_on_eof", false);

  // Get the camera parameters file
  camera_info_url_ = declare_parameter("camera_info_url", "");
  camera_name_ = declare_parameter("camera_name", "");

  // Get the image encoding
  image_encoding_ =
    declare_parameter("image_encoding", std::string(sensor_msgs::image_encodings::RGB8));
  if (image_encoding_ != sensor_msgs::image_encodings::RGB8 &&
    image_encoding_ != sensor_msgs::image_encodings::MONO8 &&
    image_encoding_ != sensor_msgs::image_encodings::YUV422 &&
    image_encoding_ != "jpeg")
  {
    RCLCPP_FATAL_STREAM(get_logger(), "Unsupported image encoding: " + image_encoding_);
    return false;
  }

  camera_info_manager_.setCameraName(camera_name_);

  if (!camera_info_url_.empty() && camera_info_manager_.validateURL(camera_info_url_) &&
    camera_info_manager_.loadCameraInfo(camera_info_url_))
  {
    RCLCPP_INFO_STREAM(get_logger(), "Loaded camera calibration from " << camera_info_url_);
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Camera info at: " << camera_info_url_ << " not found. Using an uncalibrated config.");
  }

  // Get TF Frame
  frame_id_ = declare_parameter("frame_id", "camera_frame");
  if (frame_id_ == "camera_frame") {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "No camera frame_id set, using frame \"" << frame_id_ << "\".");
  }

  use_sensor_data_qos_ = declare_parameter("use_sensor_data_qos", false);
  // A direct raw publisher avoids loading image transport plugins on Foxy,
  // whose image_transport does not support Humble's publisher allowlist.
  use_image_transport_ = declare_parameter("use_image_transport", true);

  return true;
}

bool GSCam::set_pipeline_state(GstState state)
{
  if (gst_element_set_state(pipeline_, state) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(
      get_logger(), "Could not set GStreamer state to %s.",
      gst_element_state_get_name(state));
    return false;
  }
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(startup_timeout_sec_);
  while (!stop_signal_ && rclcpp::ok(get_node_base_interface()->get_context())) {
    const auto result = gst_element_get_state(pipeline_, NULL, NULL, 100 * GST_MSECOND);
    if (result == GST_STATE_CHANGE_SUCCESS || result == GST_STATE_CHANGE_NO_PREROLL) {
      return true;
    }
    if (result == GST_STATE_CHANGE_FAILURE || std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_ERROR(
        get_logger(), "GStreamer failed or timed out entering %s within %.1f s.",
        gst_element_state_get_name(state), startup_timeout_sec_);
      return false;
    }
  }
  return false;
}

bool GSCam::init_stream()
{
  gst_init(NULL, NULL);
  RCLCPP_DEBUG_STREAM(get_logger(), "GStreamer version: " << gst_version_string());
  GError * error = NULL;
  pipeline_ = gst_parse_launch(gsconfig_.c_str(), &error);
  // gst_parse_launch can return a partial pipeline AND an error.
  if (error || !pipeline_) {
    RCLCPP_ERROR(
      get_logger(), "Could not build GStreamer pipeline: %s",
      error ? error->message : "no pipeline returned");
    if (error) {
      g_error_free(error);
    }
    return false;
  }

  sink_ = gst_element_factory_make("appsink", NULL);
  if (!sink_) {
    RCLCPP_ERROR(get_logger(), "GStreamer appsink plugin is unavailable.");
    return false;
  }
  // Retain our own reference, whether or not adding the sink to the bin succeeds.
  gst_object_ref_sink(sink_);
  configure_latest_frame_sink(GST_APP_SINK(sink_));
  GstCaps * caps = NULL;
  if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
    caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "RGB", NULL);
  } else if (image_encoding_ == sensor_msgs::image_encodings::MONO8) {
    caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "GRAY8", NULL);
  } else if (image_encoding_ == sensor_msgs::image_encodings::YUV422) {
    caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "UYVY", NULL);
  } else {
    caps = gst_caps_new_simple("image/jpeg", NULL, NULL);
  }
  gst_app_sink_set_caps(GST_APP_SINK(sink_), caps);
  gst_caps_unref(caps);
  gst_base_sink_set_sync(GST_BASE_SINK(sink_), sync_sink_);

  if (GST_IS_PIPELINE(pipeline_)) {
    GstPad * outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline_), GST_PAD_SRC);
    if (!outpad) {
      RCLCPP_ERROR(get_logger(), "gscam_config must end with an output, without a sink.");
      return false;
    }
    GstElement * output = gst_pad_get_parent_element(outpad);
    gst_object_unref(outpad);
    if (!output) {
      RCLCPP_ERROR(get_logger(), "Could not find pipeline output element.");
      return false;
    }
    const bool linked = gst_bin_add(GST_BIN(pipeline_), sink_) &&
      gst_element_link(output, sink_);
    gst_object_unref(output);
    if (!linked) {
      RCLCPP_ERROR(get_logger(), "Could not link pipeline output to appsink.");
      return false;
    }
  } else {
    GstElement * source = pipeline_;
    gst_object_ref_sink(source);
    pipeline_ = gst_pipeline_new(NULL);
    const bool added = pipeline_ && gst_bin_add(GST_BIN(pipeline_), source);
    const bool linked = added && gst_bin_add(GST_BIN(pipeline_), sink_) &&
      gst_element_link(source, sink_);
    gst_object_unref(source);
    if (!linked) {
      RCLCPP_ERROR(get_logger(), "Could not wrap source in an appsink pipeline.");
      return false;
    }
  }

  GstClock * clock = gst_system_clock_obtain();
  time_offset_ = now().nanoseconds() - static_cast<int64_t>(gst_clock_get_time(clock));
  gst_object_unref(clock);
  if (!set_pipeline_state(GST_STATE_PAUSED)) {
    return false;
  }

  const auto qos = use_sensor_data_qos_ ? rclcpp::SensorDataQoS().keep_last(1) : rclcpp::QoS{1};
  if (image_encoding_ == "jpeg") {
    jpeg_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "camera/image_raw/compressed", qos);
    cinfo_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", qos);
  } else if (!use_image_transport_) {
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("camera/image_raw", qos);
    cinfo_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", qos);
  } else {
#ifdef USE_OLD_RMW_QOS_IMAGE_TRANSPORT
    camera_pub_ = image_transport::create_camera_publisher(
      this, "camera/image_raw", qos.get_rmw_qos_profile());
#else
    camera_pub_ = image_transport::create_camera_publisher(*this, "camera/image_raw", qos);
#endif
  }
  return true;
}

void GSCam::publish_stream()
{
  RCLCPP_INFO_STREAM(get_logger(), "Publishing stream...");

  if (preroll_ && (!set_pipeline_state(GST_STATE_PLAYING) ||
    !set_pipeline_state(GST_STATE_PAUSED)))
  {
    return;
  }
  if (!set_pipeline_state(GST_STATE_PLAYING)) {
    return;
  }
  RCLCPP_INFO(get_logger(), "Started stream.");

  auto last_sample = std::chrono::steady_clock::now();
  while (!stop_signal_ && rclcpp::ok(get_node_base_interface()->get_context())) {
    // Poll briefly so shutdown does not wait for the full stream timeout.
    GstSample * sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink_), 100 * GST_MSECOND);
    if (!sample) {
      if (stop_signal_ || !rclcpp::ok(get_node_base_interface()->get_context())) {
        break;
      }
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_sample).count();
      if (!gst_app_sink_is_eos(GST_APP_SINK(sink_)) && elapsed < sample_timeout_sec_) {
        continue;
      }
      GstBus * bus = gst_element_get_bus(pipeline_);
      GstMessage * message = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR);
      if (message) {
        GError * error = NULL;
        gchar * debug = NULL;
        gst_message_parse_error(message, &error, &debug);
        RCLCPP_ERROR(
          get_logger(), "GStreamer %s: %s (%s)",
          GST_OBJECT_NAME(message->src), error->message, debug ? debug : "no details");
        g_error_free(error);
        g_free(debug);
        gst_message_unref(message);
      }
      gst_object_unref(bus);
      RCLCPP_ERROR(
        get_logger(), "No GStreamer sample within %.1f s (EOS=%s).",
        sample_timeout_sec_, gst_app_sink_is_eos(GST_APP_SINK(sink_)) ? "true" : "false");
      break;
    }
    last_sample = std::chrono::steady_clock::now();
    GstBuffer * buf = gst_sample_get_buffer(sample);
    if (!buf) {
      gst_sample_unref(sample);
      RCLCPP_ERROR(get_logger(), "GStreamer sample has no buffer.");
      break;
    }
    // Sample caps are borrowed: do not acquire a pad/caps reference per frame.
    GstCaps * caps = gst_sample_get_caps(sample);
    GstVideoInfo video_info;
    GstVideoFrame frame = {};
    GstMapInfo info = {};
    if (image_encoding_ == "jpeg") {
      if (!gst_buffer_map(buf, &info, GST_MAP_READ)) {
        gst_sample_unref(sample);
        RCLCPP_ERROR(get_logger(), "Could not map JPEG buffer.");
        break;
      }
    } else {
      if (!caps || !gst_video_info_from_caps(&video_info, caps) ||
        !gst_video_frame_map(&frame, &video_info, buf, GST_MAP_READ))
      {
        gst_sample_unref(sample);
        RCLCPP_ERROR(get_logger(), "Could not map video frame.");
        break;
      }
      width_ = GST_VIDEO_INFO_WIDTH(&video_info);
      height_ = GST_VIDEO_INFO_HEIGHT(&video_info);
    }

    // Update header information
    sensor_msgs::msg::CameraInfo cur_cinfo = camera_info_manager_.getCameraInfo();
    if (image_encoding_ != "jpeg" && !camera_info_manager_.isCalibrated()) {
      cur_cinfo.width = width_;
      cur_cinfo.height = height_;
    }
    sensor_msgs::msg::CameraInfo::SharedPtr cinfo;
    cinfo.reset(new sensor_msgs::msg::CameraInfo(cur_cinfo));
    if (use_gst_timestamps_) {
      const GstSegment * segment = gst_sample_get_segment(sample);
      const GstClockTime running_time = segment && GST_BUFFER_PTS_IS_VALID(buf) ?
        gst_segment_to_running_time(segment, GST_FORMAT_TIME, GST_BUFFER_PTS(buf)) :
        GST_CLOCK_TIME_NONE;
      if (!GST_CLOCK_TIME_IS_VALID(running_time)) {
        RCLCPP_WARN(get_logger(), "Dropping frame with invalid GStreamer timestamp.");
        if (image_encoding_ == "jpeg") {
          gst_buffer_unmap(buf, &info);
        } else {
          gst_video_frame_unmap(&frame);
        }
        gst_sample_unref(sample);
        continue;
      }
      cinfo->header.stamp = rclcpp::Time(
        static_cast<int64_t>(running_time + gst_element_get_base_time(pipeline_)) + time_offset_);
    } else {
      cinfo->header.stamp = now();
    }
    // RCLCPP_INFO(get_logger(), "Image time stamp: %.3f",cinfo->header.stamp.toSec());
    cinfo->header.frame_id = frame_id_;
    if (image_encoding_ == "jpeg") {
      sensor_msgs::msg::CompressedImage::SharedPtr img(new sensor_msgs::msg::CompressedImage());
      img->header = cinfo->header;
      img->format = "jpeg";
      img->data.resize(info.size);
      std::copy(
        info.data, info.data + info.size,
        img->data.begin());
      jpeg_pub_->publish(*img);
      cinfo_pub_->publish(*cinfo);
    } else {
      const size_t bytes_per_pixel = image_encoding_ == sensor_msgs::image_encodings::YUV422 ?
        2 : sensor_msgs::image_encodings::numChannels(image_encoding_);
      const size_t row_size = static_cast<size_t>(width_) * bytes_per_pixel;
      const size_t expected_frame_size = row_size * height_;

      // Construct Image message
      sensor_msgs::msg::Image::SharedPtr img(new sensor_msgs::msg::Image());

      img->header = cinfo->header;

      // Image data and metadata
      img->width = width_;
      img->height = height_;
      img->encoding = image_encoding_;
      img->is_bigendian = false;
      img->data.resize(expected_frame_size);

      // Copy only the data we received
      // Since we're publishing shared pointers, we need to copy the image so
      // we can free the buffer allocated by gstreamer
      img->step = row_size;
      const auto * pixels = static_cast<const guint8 *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
      const gint stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
      // GStreamer rows can include alignment padding; ROS rows are packed.
      for (int row = 0; row < height_; ++row) {
        std::memcpy(img->data.data() + row * row_size, pixels + row * stride, row_size);
      }

      // Publish the image/info
      if (use_image_transport_) {
        camera_pub_.publish(img, cinfo);
      } else {
        image_pub_->publish(*img);
        cinfo_pub_->publish(*cinfo);
      }
    }

    if (image_encoding_ == "jpeg") {
      gst_buffer_unmap(buf, &info);
    } else {
      gst_video_frame_unmap(&frame);
    }
    gst_sample_unref(sample);
  }
}

void GSCam::cleanup_stream()
{
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = NULL;
  }
  if (sink_) {
    gst_object_unref(sink_);
    sink_ = NULL;
  }
}

void GSCam::run()
{
  const auto context = get_node_base_interface()->get_context();
  try {
    if (!configure()) {
      RCLCPP_ERROR(get_logger(), "Failed to configure gscam.");
    } else {
      while (!stop_signal_ && rclcpp::ok(context)) {
        if (!init_stream()) {
          RCLCPP_ERROR(get_logger(), "Failed to initialize gscam stream.");
          break;
        }
        publish_stream();
        cleanup_stream();
        if (!reopen_on_eof_) {
          break;
        }
      }
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "gscam stopped: %s", error.what());
  }
  cleanup_stream();
  // Exiting the worker must also end the executor, allowing launch to respawn.
  rclcpp::shutdown(context);
}

// Example callbacks for appsink
// TODO(someone): enable callback-based capture
void gst_eos_cb(GstAppSink * appsink, gpointer user_data)
{
}
GstFlowReturn gst_new_preroll_cb(GstAppSink * appsink, gpointer user_data)
{
  return GST_FLOW_OK;
}
GstFlowReturn gst_new_asample_cb(GstAppSink * appsink, gpointer user_data)
{
  return GST_FLOW_OK;
}

}  // namespace gscam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gscam::GSCam)
