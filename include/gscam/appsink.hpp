// Copyright 2026 Pavlou Fire Robotics
// SPDX-License-Identifier: Apache-2.0
#ifndef GSCAM__APPSINK_HPP_
#define GSCAM__APPSINK_HPP_

#include <gst/app/gstappsink.h>

namespace gscam
{
inline void configure_latest_frame_sink(GstAppSink * sink)
{
  gst_app_sink_set_max_buffers(sink, 1);
  gst_app_sink_set_drop(sink, TRUE);
  gst_app_sink_set_wait_on_eos(sink, FALSE);
  g_object_set(sink, "enable-last-sample", FALSE, NULL);
}
}  // namespace gscam
#endif  // GSCAM__APPSINK_HPP_
