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

/** \author Tully Foote */

#include "tf2_ros/transform_listener.h"


using namespace tf2_ros;

//TODO(tfoote replace these terrible macros)
#define ROS_ERROR printf
#define ROS_FATAL printf
#define ROS_INFO printf
#define ROS_WARN printf

TransformListener::TransformListener(tf2::BufferCore& buffer, bool spin_thread):
  dedicated_listener_thread_(NULL), buffer_(buffer), using_dedicated_thread_(false)
{
  if (spin_thread)
    initWithThread();
  else
    init();
}

TransformListener::TransformListener(tf2::BufferCore& buffer, rclcpp::node::Node::SharedPtr nh, bool spin_thread)
: dedicated_listener_thread_(NULL)
, node_(nh)
, buffer_(buffer)
, using_dedicated_thread_(false)
{
  if (spin_thread)
    initWithThread();
  else
    init();
}


TransformListener::~TransformListener()
{
  using_dedicated_thread_ = false;
  if (dedicated_listener_thread_)
  {
    dedicated_listener_thread_->join();
    delete dedicated_listener_thread_;
  }
}

void test_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg){
  return;
}

void TransformListener::init()
{
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 100;
  auto standard_callback = [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {this->subscription_callback(msg);};
  auto static_callback = [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {this->static_subscription_callback(msg);};
  message_subscription_tf_ = node_->create_subscription<tf2_msgs::msg::TFMessage>("/tf", standard_callback, custom_qos_profile);
  message_subscription_tf_ = node_->create_subscription<tf2_msgs::msg::TFMessage>("/tf", static_callback, custom_qos_profile);
}

void TransformListener::initWithThread()
{
  this->init();
  //TODO(tfoote) reenable dedicated thread
  // using_dedicated_thread_ = true;
  // ros::SubscribeOptions ops_tf = ros::SubscribeOptions::create<tf2_msgs::TFMessage>("/tf", 100, boost::bind(&TransformListener::subscription_callback, this, _1), ros::VoidPtr(), &tf_message_callback_queue_); ///\todo magic number
  // message_subscription_tf_ = node_.subscribe(ops_tf);
  // 
  // ros::SubscribeOptions ops_tf_static = ros::SubscribeOptions::create<tf2_msgs::TFMessage>("/tf_static", 100, boost::bind(&TransformListener::static_subscription_callback, this, _1), ros::VoidPtr(), &tf_message_callback_queue_); ///\todo magic number
  // message_subscription_tf_static_ = node_.subscribe(ops_tf_static);
  // 
  // dedicated_listener_thread_ = new boost::thread(boost::bind(&TransformListener::dedicatedListenerThread, this));
  // 
  // //Tell the buffer we have a dedicated thread to enable timeouts
  // buffer_.setUsingDedicatedThread(true);
}



void TransformListener::subscription_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  subscription_callback_impl(msg, false);
}
void TransformListener::static_subscription_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  subscription_callback_impl(msg, true);
}

void TransformListener::subscription_callback_impl(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static)
{
  auto now = tf2::get_now();
  if(now < last_update_){
    ROS_WARN("Detected jump back in time. Clearing TF buffer.");
    buffer_.clear();
  }
  last_update_ = now;



  const tf2_msgs::msg::TFMessage& msg_in = *msg;
  //TODO(tfoote) find a way to get the authority
  std::string authority = "Authority undetectable"; //msg_evt.getPublisherName(); // lookup the authority
  for (unsigned int i = 0; i < msg_in.transforms.size(); i++)
  {
    try
    {
      buffer_.setTransform(msg_in.transforms[i], authority, is_static);
    }
    
    catch (tf2::TransformException& ex)
    {
      ///\todo Use error reporting
      std::string temp = ex.what();
      ROS_ERROR("Failure to set recieved transform from %s to %s with error: %s\n", msg_in.transforms[i].child_frame_id.c_str(), msg_in.transforms[i].header.frame_id.c_str(), temp.c_str());
    }
  }
};
