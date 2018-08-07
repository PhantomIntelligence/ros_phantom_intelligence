/**
  Copyright 2018 Phantom Intelligence Inc.

  Redistribution and use in source and binary forms,
  with or without modification, are permitted provided
  that the following conditions are met:

  1. Redistributions of source code must retain
  the above copyright notice, this list of conditions
  and the following disclaimer.

  2. Redistributions in binary form must reproduce
  the above copyright notice, this list of conditions
  and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names
  of its contributors may be used to endorse or promote
  products derived from this software without specific
  prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include "ros_phantom_intelligence/ros_communication_strategy.h"

namespace phantom_intelligence_driver
{
  using ros_communication::ROSCommunicationStrategy;

  ROSCommunicationStrategy::ROSCommunicationStrategy(std::string const& sensor_model) :
      sensor_model_(sensor_model),
      message_publisher_(node_handle_.advertise<PublicizedMessage>(sensor_model, 1000))
  {
  }

  ROSCommunicationStrategy::~ROSCommunicationStrategy() noexcept
  {
  }

  void ROSCommunicationStrategy::openConnection(std::string const& publicizedTopic)
  {
    ROS_INFO("Connecting %s sensor", sensor_model_.c_str());
  }

  void ROSCommunicationStrategy::closeConnection()
  {
    ROS_INFO("Disconnecting %s sensor", sensor_model_.c_str());
  }

  void ROSCommunicationStrategy::sendMessage(MESSAGE&& message)
  {
    if(ros::ok())
    {

      PublicizedMessage msg;

      msg.header.frame_id = std::to_string(message.frameID);

      msg.header.stamp = ros::Time::now();
      msg.header.seq++;

      message_publisher_.publish(msg);

      ros::spinOnce();
    }
  }
}
