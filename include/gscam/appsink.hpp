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
