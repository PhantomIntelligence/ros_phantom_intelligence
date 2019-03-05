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

#ifndef ROS_PHANTOM_INTELLIGENCE_EVAL_ADAL6110_16_CONNECTION_H
#define ROS_PHANTOM_INTELLIGENCE_EVAL_ADAL6110_16_CONNECTION_H

#include <sensor-gateway/application/EVAL_ADAL6110_16_AccessLink.h>

#include "ros_phantom_intelligence/sensor_connection.h"

namespace phantom_intelligence_driver
{

  namespace eval_adal6110_16
  {

      using Sensor = SensorGateway::EVAL_ADAL6110_16_AccessLink;
      using AccessLink = Sensor::AccessLink;
      using GatewayStructures = Sensor::GatewayStructures;
      using Frame = SensorConnection<GatewayStructures>::FrameMessage;

      using DataTranslationStrategy = Sensor::DataTranslationStrategy;
      using SensorCommunicationStrategy = Sensor::SensorCommunicationStrategy;

    class EVAL_ADAL6110_16_Connection final : public SensorConnection<GatewayStructures>
    {
    protected:
      using super = SensorConnection<GatewayStructures>;
      using super::assertConnectionHasNotBeenEstablished;
      using super::completeConnection;
      using super::assertConnectionHasNotBeenRuptured;
      using super::completeDisconnect;

    public:
      explicit EVAL_ADAL6110_16_Connection(std::string const& device_location);

      void start() override;

      void terminateAndJoin() override;

    private:

      using super::ros_communication_strategy_;


        DataTranslationStrategy dataTranslationStrategy;
        SensorCommunicationStrategy sensorCommunicationStrategy;

      AccessLink eval_adal6110_16_access_link_;
    };
  }
}

#endif //ROS_PHANTOM_INTELLIGENCE_EVAL_ADAL6110_16_CONNECTION_H
