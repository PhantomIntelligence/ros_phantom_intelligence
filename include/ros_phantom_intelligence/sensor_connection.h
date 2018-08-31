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

#ifndef ROS_PHANTOM_INTELLIGENCE_SENSORCONNECTION_H
#define ROS_PHANTOM_INTELLIGENCE_SENSORCONNECTION_H

#include "ros_phantom_intelligence/ros_communication_strategy.h"

namespace phantom_intelligence_driver
{

  enum SensorModel : uint32_t
  {
    AWL7,
    AWL16,
    GUARDIAN,
    UNDEFINED
  };

  template<class T>
  class SensorConnection
  {
  protected:

    const std::string PUBLICIZED_TOPIC = "SensorFrame";

  public:

    explicit SensorConnection(SensorModel sensor_model,
                              std::string const& device_location) :
        ros_communication_strategy_(fetchModelName(sensor_model))
    {
    }

    virtual void start() = 0;

    virtual void terminateAndJoin() = 0;

  protected:

    void assertConnectionHasNotBeenEstablished()
    {
      if(isSensorConnected())
      {
        ROS_WARN("This Sensor Connection has already been established!");
        // TODO: throw error and catch it
      }

      ROS_INFO("Connection...");
    }

    void completeConnection()
    {
      sensor_connected_.store(true);

      ROS_INFO("Connection complete!");
    }

    void assertConnectionHasNotBeenRuptured()
    {
      if(!isSensorConnected())
      {
        ROS_WARN("This Sensor Connection has already been ruptured!");
        // TODO: throw error and catch it
      }

      ROS_INFO("Disconnection...");
    }

    void completeDisconnect()
    {
      sensor_connected_.store(false);

      ROS_INFO("Disconnection complete!");
    }

    ros_communication::ROSCommunicationStrategy<T> ros_communication_strategy_;

  private:

    static std::string fetchModelName(SensorModel sensor_model)
    {
      switch (sensor_model)
      {
        case SensorModel::AWL7:
          return "AWL7";
          break;
        case SensorModel::AWL16:
          return "AWL16";
          break;
        case SensorModel::GUARDIAN:
          return "Guardian";
          break;
        case SensorModel::UNDEFINED:
        default:
          return "UNDEFINED";
          break;
      }
    }

    bool isSensorConnected() const
    {
      return sensor_connected_.load();
    }

    AtomicFlag sensor_connected_;

    std::string device_location_;
  };
}

#endif //ROS_PHANTOM_INTELLIGENCE_SENSORCONNECTION_H
