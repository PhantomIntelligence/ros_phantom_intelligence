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

#include "ros_phantom_intelligence/awl16_connection.h"

namespace phantom_intelligence_driver
{
  using awl16::MessageTranslationStrategy;
  using awl16::AWL16CommunicationStrategy;

  using awl16::AWL16Connection;

  AWL16Connection::AWL16Connection(std::string const& device_location) :
      super(SensorModel::AWL16, device_location),
      awl16_access_link_(&ros_communication_strategy_, &message_translation_strategy_, &awl16_communication_strategy_)
  {
  }

  void AWL16Connection::connect()
  {
    super::assertConnectionHasNotBeenEstablished();

    awl16_access_link_.connect(PUBLICIZED_TOPIC);

    super::completeConnection();
  }

  void AWL16Connection::disconnect()
  {
    super::assertConnectionHasNotBeenRuptured();

    awl16_access_link_.disconnect();

    super::completeDisconnect();
  }
}
