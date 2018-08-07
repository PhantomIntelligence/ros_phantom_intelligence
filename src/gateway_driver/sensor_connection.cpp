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

#include "ros_phantom_intelligence/sensor_connection.h"

namespace phantom_intelligence_driver
{

  SensorConnection::SensorConnection(SensorModel sensor_model, std::string const& device_location) :
      ros_communication_strategy_(fetchModelName(sensor_model))
  {
  }

  void SensorConnection::assertConnectionHasNotBeenEstablished()
  {
    if(isSensorConnected())
    {
      ROS_WARN("This Sensor Connection has already been established!");
      // TODO: throw error and catch it
    }

    ROS_INFO("Connection...");
  }

  void SensorConnection::completeConnection()
  {
    sensor_connected_.store(true);

    ROS_INFO("Connection complete!");
  }

  void SensorConnection::assertConnectionHasNotBeenRuptured()
  {
    if(!isSensorConnected())
    {
      ROS_WARN("This Sensor Connection has already been ruptured!");
      // TODO: throw error and catch it
    }

    ROS_INFO("Disconnection...");
  }

  void SensorConnection::completeDisconnect()
  {
    sensor_connected_.store(false);

    ROS_INFO("Disconnection complete!");
  }

  std::string SensorConnection::fetchModelName(SensorModel sensor_model)
  {
    switch (sensor_model)
    {
      case SensorModel::AWL7:
        return "AWL7";
        break;
      case SensorModel::AWL16:
        return "AWL16";
        break;
      case SensorModel::UNDEFINED:
      default:
        return "UNDEFINED";
        break;
    }
  }

  bool SensorConnection::isSensorConnected() const
  {
    return sensor_connected_.load();
  }
}
