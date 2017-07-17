/*
 * Copyright (c) 2012, Willow Garage, Inc.
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
#ifndef MESSAGE_FILTER_DISPLAY_H
#define MESSAGE_FILTER_DISPLAY_H

#ifndef Q_MOC_RUN
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <rclcpp/subscription.hpp>
#endif

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/ros_topic_property.h"

#include "rviz/display.h"

namespace rviz
{

/** @brief Helper superclass for MessageFilterDisplay, needed because
 * Qt's moc and c++ templates don't work nicely together.  Not
 * intended to be used directly. */
class _RosTopicDisplay: public Display
{
Q_OBJECT
public:
  _RosTopicDisplay()
    {
      topic_property_ = new RosTopicProperty( "Topic", "",
                                              "", "",
                                              this, SLOT( updateTopic() ));
      unreliable_property_ = new BoolProperty( "Unreliable", false,
                                               "Prefer UDP topic transport",
                                               this,
                                               SLOT( updateTopic() ));
    }

protected Q_SLOTS:
  virtual void updateTopic() = 0;

protected:
  RosTopicProperty* topic_property_;
  BoolProperty* unreliable_property_;
};

/** @brief Display subclass using a tf::MessageFilter, templated on the ROS message type.
 *
 * This class brings together some common things used in many Display
 * types.  It has a tf::MessageFilter to filter incoming messages, and
 * it handles subscribing and unsubscribing when the display is
 * enabled or disabled.  It also has an Ogre::SceneNode which  */
template<class MessageType>
class MessageFilterDisplay: public _RosTopicDisplay
{
// No Q_OBJECT macro here, moc does not support Q_OBJECT in a templated class.
public:
  /** @brief Convenience typedef so subclasses don't have to use
   * the long templated class name to refer to their super class. */
  typedef MessageFilterDisplay<MessageType> MFDClass;

  MessageFilterDisplay(const std::string& msgType)
    : messages_received_( 0 )
    {
      // TODO: get message type from class?
      QString message_type = QString::fromStdString( msgType );
      topic_property_->setMessageType( message_type );
      topic_property_->setDescription( message_type + " topic to subscribe to." );
    }

  virtual void onInitialize()
    {
    }

  virtual ~MessageFilterDisplay()
    {
      unsubscribe();
    }

  virtual void reset()
    {
      Display::reset();
      messages_received_ = 0;
    }

  virtual void setTopic( const QString &topic, const QString &datatype )
    {
      topic_property_->setString( topic );
    }

protected:
  virtual void updateTopic()
    {
      unsubscribe();
      reset();
      subscribe();
      context_->queueRender();
    }

  virtual void subscribe()
    {
      if( !isEnabled() )
      {
        return;
      }

      // Cannot subscribe to an empty topic
      if (topic_property_->getTopicStd().size() == 0) {
	return;
      }

      try
      {
	/* // TODO: support UDP vs TCP transport hints
        ros::TransportHints transport_hint = ros::TransportHints().reliable();
        // Determine UDP vs TCP transport for user selection.
        if (unreliable_property_->getBool())
        {
          transport_hint = ros::TransportHints().unreliable();
        }
	*/

	sub_ = update_nh_->create_subscription<MessageType>(
            topic_property_->getTopicStd(),
	    10,
            std::bind(&MessageFilterDisplay::incomingMessage, this, std::placeholders::_1));
        setStatus( StatusProperty::Ok, "Topic", "OK" );
      }
      catch( rclcpp::exceptions::RCLError& e )
      {
        setStatus( StatusProperty::Error, "Topic", QString( "Error subscribing: " ) + e.what() );
      }
    }

  virtual void unsubscribe()
    {
      sub_.reset();
    }

  virtual void onEnable()
    {
      subscribe();
    }

  virtual void onDisable()
    {
      unsubscribe();
      reset();
    }

  virtual void fixedFrameChanged()
    {
      reset();
    }

  /** @brief Incoming message callback.  Checks if the message pointer
   * is valid, increments messages_received_, then calls
   * processMessage(). */
  void incomingMessage( const typename MessageType::SharedPtr msg )
    {
      if( !msg )
      {
        return;
      }

      // Skip any messages that can not be transformed to the fixed frame
      tf2_ros::Buffer* buffer = context_->getFrameManager()->getTFBuffer();
      if (!buffer->canTransform(fixed_frame_.toStdString(),
				getMsgFrame(msg),
				getMsgTime(msg),
				tf2::durationFromSec(0.1)))
      {
	return;
      }

      ++messages_received_;
      setStatus( StatusProperty::Ok, "Topic", QString::number( messages_received_ ) + " messages received" );

      processMessage( msg );
    }

  /** @brief Implement this to process the contents of a message.
   *
   * This is called by incomingMessage(). */
  virtual void processMessage( const typename MessageType::SharedPtr msg ) = 0;

  /** @brief Get the frame for the given message. */
  virtual std::string getMsgFrame(const typename MessageType::SharedPtr msg) = 0;

  /** @brief Get the time stamp for the given message. */
  virtual tf2::TimePoint getMsgTime(const typename MessageType::SharedPtr msg) = 0;

  typename rclcpp::subscription::Subscription<MessageType>::SharedPtr sub_;
  uint32_t messages_received_;
};

} // end namespace rviz

#endif // MESSAGE_FILTER_DISPLAY_H
