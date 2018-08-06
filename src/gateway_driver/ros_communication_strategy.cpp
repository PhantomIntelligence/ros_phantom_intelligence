/**
	Copyright 2014-2018 Phantom Intelligence Inc.

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

#include "ros_phantom_intelligence/ros_communication_strategy.h"

namespace phantom_intelligence_driver
{
  using ros_communication::ROSCommunicationStrategy;

  ROSCommunicationStrategy::ROSCommunicationStrategy(std::string const& sensor_model) :
      sensor_model_(sensor_model),
      loop_rate_(ROS_LOOP_RATE),
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

      ROS_INFO("%s", msg.header.frame_id.c_str());

      message_publisher_.publish(msg);

      ros::spinOnce();
      loop_rate_.sleep();
    }
  }
}
