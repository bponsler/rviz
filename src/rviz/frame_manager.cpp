/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "frame_manager.h"
#include "display.h"
#include "properties/property.h"

//#include <tf/transform_listener.h>  // TODO: need this header
#include <rclcpp/rclcpp.hpp>
#include <ros2_console/console.hpp>

#include <std_msgs/msg/float32.hpp>

namespace rviz
{

FrameManager::FrameManager(boost::shared_ptr<tf2_ros::TransformListener> tf)
  :
  buffer_(tf2::Duration(10*60))
{
  if (!tf) {
    tf_.reset(new tf2_ros::TransformListener(
        buffer_,
	rclcpp::node::Node::make_shared("rviz_frame_manager"),
	true));
  }
  else {
    tf_ = tf;
  }

  setSyncMode( SyncOff );
  setPause(false);
}

FrameManager::~FrameManager()
{
}

void FrameManager::update()
{
  boost::mutex::scoped_lock lock(cache_mutex_);
  if ( !pause_ )
  {
    cache_.clear();
  }

  if ( !pause_ )
  {
    switch ( sync_mode_ )
    {
      case SyncOff:
        sync_time_ = tf2::get_now();
        break;
      case SyncExact:
        break;
      case SyncApprox:
        // adjust current time offset to sync source
        current_delta_ = 0.7*current_delta_ + 0.3*sync_delta_;
        try
        {
          sync_time_ = tf2::get_now()-tf2::durationFromSec(current_delta_);
        }
        catch (...)
        {
          sync_time_ = tf2::get_now();
        }
        break;
    }
  }
}

void FrameManager::setFixedFrame(const std::string& frame)
{
  bool emit = false;
  {
    boost::mutex::scoped_lock lock(cache_mutex_);
    if( fixed_frame_ != frame )
    {
      fixed_frame_ = frame;
      cache_.clear();
      emit = true;
    }
  }
  if( emit )
  {
    // This emission must be kept outside of the mutex lock to avoid deadlocks.
    Q_EMIT fixedFrameChanged();
  }
}

void FrameManager::setPause( bool pause )
{
  pause_ = pause;
}

void FrameManager::setSyncMode( SyncMode mode )
{
  sync_mode_ = mode;
  sync_time_ = tf2::TimePointZero;
  current_delta_ = 0;
  sync_delta_ = 0;
}

void FrameManager::syncTime( tf2::TimePoint time )
{
  switch ( sync_mode_ )
  {
    case SyncOff:
      break;
    case SyncExact:
      sync_time_ = time;
      break;
    case SyncApprox:
      if ( time == tf2::TimePointZero )
      {
        sync_delta_ = 0;
        return;
      }
      // avoid exception due to negative time
      if ( tf2::get_now() >= time )
      {
        sync_delta_ = tf2::durationToSec(tf2::get_now() - time);
      }
      else
      {
        setSyncMode( SyncApprox );
      }
      break;
  }
}

bool FrameManager::adjustTime( const std::string &frame, tf2::TimePoint& time )
{
  // we only need to act if we get a zero timestamp, which means "latest"
  if ( time != tf2::TimePointZero )
  {
    return true;
  }

  tf2::CompactFrameID fixed_id = buffer_._lookupFrameNumber(fixed_frame_);
  tf2::CompactFrameID frame_id = buffer_._lookupFrameNumber(frame);
  
  switch ( sync_mode_ )
  {
    case SyncOff:
      break;
    case SyncExact:
      time = sync_time_;
      break;
    case SyncApprox:
      {    
        // if we don't have tf info for the given timestamp, use the latest available
        tf2::TimePoint latest_time;
        std::string error_string;
	tf2::TF2Error error_code = buffer_._getLatestCommonTime( fixed_id, frame_id, latest_time, &error_string );
        if ( error_code != tf2::TF2Error::NO_ERROR )
        {
          ROS_ERROR("Error getting latest time from frame '%s' to frame '%s': %s (Error code: %d)", frame.c_str(), fixed_frame_.c_str(), error_string.c_str(), (int)error_code);
          return false;
        }

        if ( latest_time > sync_time_ )
        {
          time = sync_time_;
        }
      }
      break;
  }
  return true;
}



bool FrameManager::getTransform(const std::string& frame, tf2::TimePoint time, Ogre::Vector3& position, Ogre::Quaternion& orientation)
{
  if ( !adjustTime(frame, time) )
  {
    return false;
  }

  boost::mutex::scoped_lock lock(cache_mutex_);

  position = Ogre::Vector3(9999999, 9999999, 9999999);
  orientation = Ogre::Quaternion::IDENTITY;

  if (fixed_frame_.empty())
  {
    return false;
  }

  M_Cache::iterator it = cache_.find(CacheKey(frame, time));
  if (it != cache_.end())
  {
    position = it->second.position;
    orientation = it->second.orientation;
    return true;
  }

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0f;

  if (!transform(frame, time, pose, position, orientation))
  {
    return false;
  }

  cache_.insert(std::make_pair(CacheKey(frame, time), CacheEntry(position, orientation)));

  return true;
}

bool FrameManager::transform(const std::string& frame, tf2::TimePoint time, const geometry_msgs::msg::Pose& pose_msg, Ogre::Vector3& position, Ogre::Quaternion& orientation)
{
  if ( !adjustTime(frame, time) )
  {
    return false;
  }

  position = Ogre::Vector3::ZERO;
  orientation = Ogre::Quaternion::IDENTITY;

  // put all pose data into a tf stamped pose
  tf2::Quaternion bt_orientation(pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w);
  tf2::Vector3 bt_position(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);

  if (bt_orientation.x() == 0.0 && bt_orientation.y() == 0.0 && bt_orientation.z() == 0.0 && bt_orientation.w() == 0.0)
  {
    bt_orientation.setW(1.0);
  }

  tf2::Transform pose_in(bt_orientation,bt_position);
  tf2::Transform pose_out;
   
  // convert pose into new frame
  try
  {
    // Look up the laser's pose that is centered
    geometry_msgs::msg::TransformStamped transMsg = buffer_.lookupTransform(
        fixed_frame_,
	frame,
	time,
	tf2::durationFromSec(0.1));

    // Convert message to tf2 type
    tf2::Transform transform(
        tf2::Quaternion(
            transMsg.transform.rotation.x,
	    transMsg.transform.rotation.y,
	    transMsg.transform.rotation.z,
	    transMsg.transform.rotation.w),
	tf2::Vector3(
	    transMsg.transform.translation.x,
	    transMsg.transform.translation.y,
	    transMsg.transform.translation.z));
    
    // Transform the position
    pose_out = transform * pose_in;
  }
  catch(std::runtime_error& e)
  {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s': %s", frame.c_str(), fixed_frame_.c_str(), e.what());
    return false;
  }

  bt_position = pose_out.getOrigin();
  position = Ogre::Vector3(bt_position.x(), bt_position.y(), bt_position.z());

  bt_orientation = pose_out.getRotation();
  orientation = Ogre::Quaternion( bt_orientation.w(), bt_orientation.x(), bt_orientation.y(), bt_orientation.z() );
  
  return true;
}

bool FrameManager::frameHasProblems(const std::string& frame, tf2::TimePoint time, std::string& error)
{
  if (!buffer_._frameExists(frame))
  {
    error = "Frame [" + frame + "] does not exist";
    if (frame == fixed_frame_)
    {
      error = "Fixed " + error;
    }
    return true;
  }

  return false;
}

bool FrameManager::transformHasProblems(const std::string& frame, tf2::TimePoint time, std::string& error)
{
  if ( !adjustTime(frame, time) )
  {
    return false;
  }

  std::string tf_error;
  bool transform_succeeded = buffer_.canTransform(fixed_frame_, frame, time, &tf_error);
  if (transform_succeeded)
  {
    return false;
  }

  bool ok = true;
  ok = ok && !frameHasProblems(fixed_frame_, time, error);
  ok = ok && !frameHasProblems(frame, time, error);

  if (ok)
  {
    std::stringstream ss;
    ss << "No transform to fixed frame [" << fixed_frame_ << "].  TF error: [" << tf_error << "]";
    error = ss.str();
    ok = false;
  }

  {
    std::stringstream ss;
    ss << "For frame [" << frame << "]: " << error;
    error = ss.str();
  }

  return !ok;
}

std::string getTransformStatusName(const std::string& caller_id)
{
  std::stringstream ss;
  ss << "Transform [sender=" << caller_id << "]";
  return ss.str();
}

/* // TODO: fix this
std::string FrameManager::discoverFailureReason(const std::string& frame_id, const tf2::TimePoint& stamp, const std::string& caller_id, tf2_rod::FilterFailureReason reason)
{
  if (reason == tf::filter_failure_reasons::OutTheBack)
  {
    std::stringstream ss;
    ss << "Message removed because it is too old (frame=[" << frame_id << "], stamp=[" << stamp << "])";
    return ss.str();
  }
  else
  {
    std::string error;
    if (transformHasProblems(frame_id, stamp, error))
    {
      return error;
    }
  }

  return "Unknown reason for transform failure";
}
*/

void FrameManager::messageArrived( const std::string& frame_id, const tf2::TimePoint& stamp,
                                   const std::string& caller_id, Display* display )
{
  display->setStatusStd( StatusProperty::Ok, getTransformStatusName( caller_id ), "Transform OK" );
}

/* // TODO: fix this
void FrameManager::messageFailed( const std::string& frame_id, const tf2::TimePoint& stamp,
                                  const std::string& caller_id, tf::FilterFailureReason reason, Display* display )
{
  std::string status_name = getTransformStatusName( caller_id );
  std::string status_text = discoverFailureReason( frame_id, stamp, caller_id, reason );

  display->setStatusStd(StatusProperty::Error, status_name, status_text );
}
*/
  
}
