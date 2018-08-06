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

#ifndef ROS_PHANTOM_INTELLIGENCE_AWL_CONNECTION_H
#define ROS_PHANTOM_INTELLIGENCE_AWL_CONNECTION_H

#include "spirit-sensor-gateway/application/SensorAccessLink.hpp"

#include "ros_phantom_intelligence/ros_communication_strategy.h"
#include "spirit-sensor-gateway/message-translation/AWLMessageToSpiritMessageTranslationStrategy.h"
#include "spirit-sensor-gateway/sensor-communication/KvaserCanCommunicationStrategy.h"

namespace phantom_intelligence_driver
{

  enum SensorModel : uint32_t
  {
    AWL7,
    AWL16
  };

  namespace awl
  {

    using ros_communication::FrameMessage;
    using RawSensorMessage = DataFlow::AWLMessage;
    using AWLAccessLink = SpiritSensorGateway::SensorAccessLink<RawSensorMessage, FrameMessage>;

    using ROSCommunicationStrategy = ros_communication::ROSCommunicationStrategy;
    using MessageTranslationStrategy = MessageTranslation::AWLMessageToSpiritMessageTranslationStrategy;
    using AWLCommunicationStrategy = SensorCommunication::KvaserCanCommunicationStrategy;


    class AWLConnection
    {
    protected:
      const std::string PUBLICIZED_TOPIC = "SpiritFrame";

    public:
      explicit AWLConnection(SensorModel sensor_model, std::string const& device_location);

      void connect();

      void disconnect();

      static std::string fetchModelName(SensorModel sensor_model);

    private:

      bool isSensorConnected() const;

      ROSCommunicationStrategy ros_communication_strategy_;
      MessageTranslationStrategy message_translation_strategy_;
      AWLCommunicationStrategy awl_communication_strategy_;

      AWLAccessLink awl_access_link_;

      AtomicFlag sensor_connected_;

      std::string device_location_;
    };
  }
}

#endif //ROS_PHANTOM_INTELLIGENCE_AWL_CONNECTION_H
