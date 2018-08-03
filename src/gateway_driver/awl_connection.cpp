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
  using awl::AWLCommunicationStrategy;
  using awl::RawSensorMessage;
  using awl::MessageTranslationStrategy;
  using awl::AWLAccessLink;
  using awl::ROSCommunicationStrategy;
  using awl::AWLConnection;

  //  AWLConnection::AWLConnection(ROSCommunicationStrategy* ros_communication_strategy) :
  //      ros_communication_strategy_(ros_communication_strategy)
  //  {
  //  }
  AWLConnection::AWLConnection()
  {
  }

  void AWLConnection::connect()
  {
    if(isSensorConnected())
    {
      // TODO: throw error and catch it
    }
    //    awl_access_link_ = new AWLAccessLink(ros_communication_strategy_,
    //                                         &message_translation_strategy_,
    //                                         &awl_communication_strategy_);
    awl_access_link_ = new AWLAccessLink(&server_communication_strategy_,
                                         &message_translation_strategy_,
                                         &awl_communication_strategy_);
    std::string server_address = "ws://localhost:8080/connect-gateway";
    ROS_INFO("Connecting to sensor... ");
    ROS_INFO("Connecting to server at %s", server_address.c_str());
    awl_access_link_->connect(server_address);
    ROS_INFO("Connected!");
    sensor_connected_.store(true);
  }

  void AWLConnection::disconnect()
  {
    if(!isSensorConnected())
    {
      // TODO: throw error and catch it
    }
    awl_access_link_->disconnect();
    sensor_connected_.store(false);
    delete awl_access_link_;
  }

  bool AWLConnection::isSensorConnected() const
  {
    return sensor_connected_.load();
  }
}
