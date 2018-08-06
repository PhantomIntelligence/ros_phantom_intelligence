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

#ifndef ROS_PHANTOM_INTELLIGENCE_ROSCOMMUNICATIONSTRATEGY_H
#define ROS_PHANTOM_INTELLIGENCE_ROSCOMMUNICATIONSTRATEGY_H

#include <ros/ros.h>
#include "phantom_intelligence/SpiritFrame.h"

#include "spirit-sensor-gateway/server-communication/ServerCommunicationStrategy.hpp"

namespace phantom_intelligence_driver
{

  namespace ros_communication
  {

    using FrameMessage = DataFlow::Frame;

    class ROSCommunicationStrategy : public ServerCommunication::ServerCommunicationStrategy<FrameMessage>
    {

    protected:

      using PublicizedMessage = phantom_intelligence::SpiritFrame;

      using super = ServerCommunication::ServerCommunicationStrategy<FrameMessage>;

      using super::MESSAGE;

      const int ROS_LOOP_RATE = 10;

    public:

      explicit ROSCommunicationStrategy(std::string const& sensor_model);

      ~ROSCommunicationStrategy() noexcept final;

      ROSCommunicationStrategy(ROSCommunicationStrategy const& other) = delete;

      ROSCommunicationStrategy(ROSCommunicationStrategy&& other) noexcept = delete;

      ROSCommunicationStrategy& operator=(ROSCommunicationStrategy const& other)& = delete;

      ROSCommunicationStrategy& operator=(ROSCommunicationStrategy&& other) noexcept = delete;

      void openConnection(std::string const& publicizedTopic) override;

      void closeConnection() override;

      void sendMessage(MESSAGE&& message) override;


    private:

      std::string sensor_model_;

      ros::NodeHandle node_handle_;
      ros::Rate loop_rate_;
      ros::Publisher message_publisher_;
    };
  }

}

#endif //ROS_PHANTOM_INTELLIGENCE_ROSCOMMUNICATIONSTRATEGY_H
