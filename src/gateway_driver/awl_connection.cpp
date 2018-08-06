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

#include <ros/ros.h>

#include "ros_phantom_intelligence/awl_connection.h"

namespace phantom_intelligence_driver
{
  using awl::RawSensorMessage;
  using awl::AWLAccessLink;

  using awl::ROSCommunicationStrategy;
  using awl::MessageTranslationStrategy;
  using awl::AWLCommunicationStrategy;

  using awl::AWLConnection;

  AWLConnection::AWLConnection(SensorModel sensor_model, std::string const& device_location) :
      ros_communication_strategy_(fetchModelName(sensor_model)),
      awl_access_link_(&ros_communication_strategy_, &message_translation_strategy_, &awl_communication_strategy_)
  {
  }

  void AWLConnection::connect()
  {
    if(isSensorConnected())
    {
      // TODO: throw error and catch it
    } else
    {

      ROS_INFO("Connection...");

      awl_access_link_.connect(PUBLICIZED_TOPIC);
      sensor_connected_.store(true);

      ROS_INFO("Connection complete!");
    }
  }

  void AWLConnection::disconnect()
  {
    if(!isSensorConnected())
    {
      // TODO: throw error and catch it
    }

    ROS_INFO("Disconnection...");

    awl_access_link_.disconnect();
    sensor_connected_.store(false);

    ROS_INFO("Disconnection complete!");
  }

  std::string AWLConnection::fetchModelName(SensorModel sensor_model)
  {
    switch (sensor_model)
    {
      case SensorModel::AWL7:
        return "AWL7";
        break;
      case SensorModel::AWL16:
        return "AWL16";
        break;
      default:
        return "Unrecognized sensor";
        break;
    }
  }

  bool AWLConnection::isSensorConnected() const
  {
    return sensor_connected_.load();
  }
}
