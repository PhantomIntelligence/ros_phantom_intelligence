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

#ifndef ROS_PHANTOM_INTELLIGENCE_AWL16CONNECTION_H
#define ROS_PHANTOM_INTELLIGENCE_AWL16CONNECTION_H

#include "ros_phantom_intelligence/sensor_connection.h"

#include "sensor-gateway/application/SensorAccessLink.hpp"
#include "sensor-gateway/message-translation/AWLTranslationStrategy.h"
#include "sensor-gateway/sensor-communication/KvaserCanCommunicationStrategy.h"

namespace phantom_intelligence_driver
{

  namespace awl16
  {

    using ros_communication::FrameMessage;
    using RawSensorMessage = DataFlow::AWLMessage;
    using AWL16AccessLink = SensorGateway::SensorAccessLink<RawSensorMessage, FrameMessage>;

    using MessageTranslationStrategy = MessageTranslation::AWLTranslationStrategy;
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
