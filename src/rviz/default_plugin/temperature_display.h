/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 */

#ifndef RVIZ_TEMPERATURE_DISPLAY_H
#define RVIZ_TEMPERATURE_DISPLAY_H

#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "rviz/message_filter_display.h"

namespace rviz
{

class IntProperty;
class PointCloudCommon;

/**
 * \class TemperatureDisplay
 * \brief Displays a Temperature message of type sensor_msgs::msg::Temperature
 *
 */
class TemperatureDisplay: public MessageFilterDisplay<sensor_msgs::msg::Temperature>
{
Q_OBJECT
public:
  TemperatureDisplay();
  ~TemperatureDisplay();

  virtual void reset();

  virtual void update( float wall_dt, float ros_dt );

private Q_SLOTS:
  void updateQueueSize();

protected:
  /** @brief Do initialization. Overridden from MessageFilterDisplay. */
  virtual void onInitialize();

  /** @brief Process a single message.  Overridden from MessageFilterDisplay. */
  virtual void processMessage( const sensor_msgs::msg::Temperature::SharedPtr msg );

  /** @brief Get the frame for the given message. */
  virtual std::string getMsgFrame(const sensor_msgs::msg::Temperature::SharedPtr msg);

  /** @brief Get the time stamp for the given message. */
  virtual tf2::TimePoint getMsgTime(const sensor_msgs::msg::Temperature::SharedPtr msg);
  
  IntProperty* queue_size_property_;

  PointCloudCommon* point_cloud_common_;
};

} // namespace rviz

#endif
