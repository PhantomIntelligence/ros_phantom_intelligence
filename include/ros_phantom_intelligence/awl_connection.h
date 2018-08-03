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

#ifndef ROS_PHANTOM_INTELLIGENCE_GATEWAY_CONNECTION_H
#define ROS_PHANTOM_INTELLIGENCE_GATEWAY_CONNECTION_H

#include "spirit-sensor-gateway/application/SensorAccessLink.hpp"
#include "spirit-sensor-gateway/sensor-communication/KvaserCanCommunicationStrategy.h"
#include "spirit-sensor-gateway/message-translation/AWLMessageToSpiritMessageTranslationStrategy.h"
#include "spirit-sensor-gateway/server-communication/ServerCommunicationStrategy.hpp"

namespace phantom_intelligence_driver
{

  using FormatedFrameMessage = DataFlow::Frame;

  namespace awl
  {

    using RawSensorMessage = DataFlow::AWLMessage;
    using AWLAccessLink = SpiritSensorGateway::SensorAccessLink<RawSensorMessage, FormatedFrameMessage>;

    using ROSCommunicationStrategy = ServerCommunication::ServerCommunicationStrategy<FormatedFrameMessage>;
    using MessageTranslationStrategy = MessageTranslation::AWLMessageToSpiritMessageTranslationStrategy;
    using AWLCommunicationStrategy = SensorCommunication::KvaserCanCommunicationStrategy;


    class AWLConnection
    {
    public:
      explicit AWLConnection(ROSCommunicationStrategy* ros_communication_strategy);

      void connect();

      void disconnect();

    private:

      bool isSensorConnected() const;

      ROSCommunicationStrategy* ros_communication_strategy_;
      MessageTranslationStrategy message_translation_strategy_;
      AWLCommunicationStrategy awl_communication_strategy_;

      AWLAccessLink* awl_access_link_;

      AtomicFlag sensor_connected_;
    };
  }
}

#endif //ROS_PHANTOM_INTELLIGENCE_GATEWAY_CONNECTION_H
