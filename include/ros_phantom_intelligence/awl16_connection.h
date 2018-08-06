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

#ifndef ROS_PHANTOM_INTELLIGENCE_AWL16CONNECTION_H
#define ROS_PHANTOM_INTELLIGENCE_AWL16CONNECTION_H

#include "ros_phantom_intelligence/sensor_connection.h"

#include "spirit-sensor-gateway/application/SensorAccessLink.hpp"
#include "spirit-sensor-gateway/message-translation/AWLMessageToSpiritMessageTranslationStrategy.h"
#include "spirit-sensor-gateway/sensor-communication/KvaserCanCommunicationStrategy.h"

namespace phantom_intelligence_driver
{

  namespace awl16
  {

    using ros_communication::FrameMessage;
    using RawSensorMessage = DataFlow::AWLMessage;
    using AWL16AccessLink = SpiritSensorGateway::SensorAccessLink<RawSensorMessage, FrameMessage>;

    using MessageTranslationStrategy = MessageTranslation::AWLMessageToSpiritMessageTranslationStrategy;
    using AWL16CommunicationStrategy = SensorCommunication::KvaserCanCommunicationStrategy;

    class AWL16Connection final : public SensorConnection
    {
    protected:
      using super = SensorConnection;

    public:
      explicit AWL16Connection(std::string const& device_location);

      void connect() override;

      void disconnect() override;

    private:

      using super::ros_communication_strategy_;
      MessageTranslationStrategy message_translation_strategy_;
      AWL16CommunicationStrategy awl16_communication_strategy_;

      AWL16AccessLink awl16_access_link_;
    };
  }
}

#endif //ROS_PHANTOM_INTELLIGENCE_AWL16CONNECTION_H
