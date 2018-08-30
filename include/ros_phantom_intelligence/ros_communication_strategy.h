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

#include "phantom_intelligence/PhantomIntelligenceHeader.h"

#include "phantom_intelligence/PhantomIntelligenceRawData.h"

#include "phantom_intelligence/PhantomIntelligenceFrame.h"
#include "phantom_intelligence/PhantomIntelligencePixel.h"
#include "phantom_intelligence/PhantomIntelligenceTrack.h"

namespace phantom_intelligence_driver
{

  namespace ros_communication
  {

    template<class T>
    class ROSCommunicationStrategy : public ServerCommunication::ServerCommunicationStrategy<T>
    {

    protected:

      using super = ServerCommunication::ServerCommunicationStrategy<T>;

      using Message = typename super::Message;
      using RawData = typename super::RawData;

      using ROSRawData = phantom_intelligence::PhantomIntelligenceRawData;

      using ROSFrame = phantom_intelligence::PhantomIntelligenceFrame;
      using ROSPixel = phantom_intelligence::PhantomIntelligencePixel;
      using ROSTrack = phantom_intelligence::PhantomIntelligenceTrack;

    public:

      static std::string fetchMessageTopicName(std::string const& sensor_topic_name) noexcept
      {
        std::ostringstream message_topic_name(sensor_topic_name, std::ios_base::ate);
        message_topic_name << "_message";
        return message_topic_name.str();
      }

      static std::string fetchRawDataTopicName(std::string const& sensor_topic_name) noexcept
      {
        std::ostringstream fetch_raw_data_name(sensor_topic_name, std::ios_base::ate);
        fetch_raw_data_name << "_raw_data";
        return fetch_raw_data_name.str();
      }

      explicit ROSCommunicationStrategy(std::string const& sensor_model) :
          sensor_model_(sensor_model),
          message_publisher_(node_handle_.advertise<ROSFrame>(
              fetchMessageTopicName(sensor_model), 1000)
          ),
          raw_data_publisher_(node_handle_.advertise<ROSRawData>(
              fetchRawDataTopicName(sensor_model), 1000)
          )
      {
      }

      ~ROSCommunicationStrategy() noexcept final
      {
      }

      ROSCommunicationStrategy(ROSCommunicationStrategy const& other) = delete;

      ROSCommunicationStrategy(ROSCommunicationStrategy&& other) noexcept = delete;

      ROSCommunicationStrategy& operator=(ROSCommunicationStrategy const& other)& = delete;

      ROSCommunicationStrategy& operator=(ROSCommunicationStrategy&& other) noexcept = delete;

      void openConnection(std::string const& publicizedTopic) override
      {
        ROS_INFO("Connecting %s sensor", sensor_model_.c_str());
      }

      void sendMessage(Message&& message) override
      {
        if(ros::ok())
        {

          ROSFrame msg;

          msg.header.header.stamp = ros::Time::now();
          msg.header.header.seq++;
          msg.header.system_id = message.systemID;

          msg.id = message.frameID;

          auto pixels = *message.getPixels();
          uint16_t const number_of_pixels = pixels.size();
          msg.pixels.reserve(number_of_pixels);

          for (auto pixel_iterator = 0u; pixel_iterator < number_of_pixels; ++pixel_iterator)
          {

            ROSPixel current_pixel;

            current_pixel.id = pixel_iterator;

            auto tracks = *pixels[pixel_iterator].getTracks();

            // For performance reasons, the Gateway sometimes output empty data.
            // Those won't be published in the ROS Topic
            auto number_of_tracks = pixels[pixel_iterator].getCurrentNumberOfTracksInPixel();

            if(number_of_tracks > 0)
            {

              current_pixel.tracks.reserve(number_of_tracks);

              for (auto track_iterator = 0u; track_iterator < number_of_tracks; ++track_iterator)
              {

                ROSTrack current_track;
                auto track_from_message = &tracks[track_iterator];

                current_track.id = track_from_message->ID;
                current_track.confidence_level = track_from_message->confidenceLevel;
                current_track.intensity = track_from_message->intensity;
                current_track.acceleration = track_from_message->acceleration;
                current_track.distance = track_from_message->distance;
                current_track.speed = track_from_message->speed;

                current_pixel.tracks.push_back(current_track);
              }
            }

            msg.pixels.push_back(current_pixel);
          }

          message_publisher_.publish(msg);

          ros::spinOnce();
        }
      }

      void sendRawData(RawData&& raw_data) override
      {
        if(ros::ok())
        {

          ROSRawData msg;

          msg.header.header.stamp = ros::Time::now();
          msg.header.header.seq++;

          auto raw_data_content = raw_data.content;
          uint16_t const number_of_values = raw_data_content.size();
          msg.data.reserve(number_of_values);

          std::copy_n(raw_data_content.begin(), number_of_values, msg.data.begin());

          raw_data_publisher_.publish(msg);

          ros::spinOnce();
        }
      }

      void closeConnection() override
      {
        ROS_INFO("Disconnecting %s sensor", sensor_model_.c_str());
      }

    private:

      std::string sensor_model_;

      ros::NodeHandle node_handle_;
      ros::Publisher message_publisher_;
      ros::Publisher raw_data_publisher_;
    };
  }
}

#endif //ROS_PHANTOM_INTELLIGENCE_ROSCOMMUNICATIONSTRATEGY_H
