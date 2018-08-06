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

#include "spirit-sensor-gateway/server-communication/ServerCommunicationStrategy.hpp"

namespace phantom_intelligence_driver
{

  using FormatedFrameMessage = DataFlow::Frame;

  namespace ServerCommunication
  {
    enum SensorModel
    {
      AWL7,
      AWL16
    };

    class ROSCommunicationStrategy final : public ServerCommunicationStrategy<FormatedFrameMessage>
    {
      explicit ROSCommunicationStrategy(SensorModel sensor_model, std::string device_location);

      ~ROSCommunicationStrategy() noexcept final;

      ROSCommunicationStrategy(ROSCommunicationStrategy const& other) = delete;

      ROSCommunicationStrategy(ROSCommunicationStrategy&& other) noexcept = delete;

      ROSCommunicationStrategy& operator=(ROSCommunicationStrategy const& other)& = delete;

      ROSCommunicationStrategy& operator=(ROSCommunicationStrategy&& other) noexcept = delete;
    };
  }

  typedef ServerCommunication::SensorModel SensorModel;
}

#endif //ROS_PHANTOM_INTELLIGENCE_ROSCOMMUNICATIONSTRATEGY_H
