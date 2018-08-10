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

#ifndef ROS_PHANTOM_INTELLIGENCE_ROSCOMMUNICATIONSTRATEGY_H
#define ROS_PHANTOM_INTELLIGENCE_ROSCOMMUNICATIONSTRATEGY_H

#include <ros/ros.h>
#include <sensor-gateway/server-communication/ServerCommunicationStrategy.hpp>

namespace phantom_intelligence_driver
{

  namespace ros_communication
  {

    using FrameMessage = DataFlow::Frame;

    class ROSCommunicationStrategy : public ServerCommunication::ServerCommunicationStrategy<FrameMessage>
    {

    protected:

      using super = ServerCommunication::ServerCommunicationStrategy<FrameMessage>;

      using super::MESSAGE;

    public:

      explicit ROSCommunicationStrategy(std::string const& sensor_model);

      ~ROSCommunicationStrategy() noexcept final;

      ROSCommunicationStrategy(ROSCommunicationStrategy const& other) = delete;

      ROSCommunicationStrategy(ROSCommunicationStrategy&& other) noexcept = delete;

      ROSCommunicationStrategy& operator=(ROSCommunicationStrategy const& other)& = delete;

      ROSCommunicationStrategy& operator=(ROSCommunicationStrategy&& other) noexcept = delete;

      void openConnection(std::string const& publicizedTopic) override;

      void closeConnection() override;

      void sendMessage(MESSAGE&& message) override;


    private:

      std::string sensor_model_;

      ros::NodeHandle node_handle_;
      ros::Publisher message_publisher_;
    };
  }

}

#endif //ROS_PHANTOM_INTELLIGENCE_ROSCOMMUNICATIONSTRATEGY_H
