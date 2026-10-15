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

#include "gscam/appsink.hpp"
#include "gtest/gtest.h"

// A stalled consumer must retain the final frame, not the first queued frame.
TEST(AppSinkTest, retains_latest_frame_without_consumer)
{
  gst_init(nullptr, nullptr);
  GstElement * pipeline = gst_parse_launch(
    "videotestsrc num-buffers=100 ! video/x-raw,framerate=30/1 "
    "! appsink name=capture sync=false", nullptr);
  ASSERT_NE(pipeline, nullptr);
  GstElement * sink = gst_bin_get_by_name(GST_BIN(pipeline), "capture");
  gscam::configure_latest_frame_sink(GST_APP_SINK(sink));
  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  GstBus * bus = gst_element_get_bus(pipeline);
  GstMessage * message = gst_bus_timed_pop_filtered(
    bus, 5 * GST_SECOND,
    static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
  EXPECT_NE(message, nullptr);
  if (message) {
    EXPECT_EQ(GST_MESSAGE_TYPE(message), GST_MESSAGE_EOS);
    gst_message_unref(message);
  }
  GstSample * sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), GST_SECOND);
  EXPECT_NE(sample, nullptr);
  if (sample) {
    EXPECT_GE(GST_BUFFER_PTS(gst_sample_get_buffer(sample)), 98 * GST_SECOND / 30);
    gst_sample_unref(sample);
  }
  sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), 0);
  EXPECT_EQ(sample, nullptr);
  if (sample) {
    gst_sample_unref(sample);
  }
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(bus);
  gst_object_unref(sink);
  gst_object_unref(pipeline);
}
