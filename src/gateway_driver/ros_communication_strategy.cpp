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

#include "ros_phantom_intelligence/ros_communication_strategy.h"

#include <iterator>
#include "phantom_intelligence/PhantomIntelligenceFrame.h"
#include "phantom_intelligence/PhantomIntelligencePixel.h"
#include "phantom_intelligence/PhantomIntelligenceTrack.h"

namespace phantom_intelligence_driver
{
  using ros_communication::ROSCommunicationStrategy;


  using Frame = phantom_intelligence::PhantomIntelligenceFrame;
  using Pixel = phantom_intelligence::PhantomIntelligencePixel;
  using Track = phantom_intelligence::PhantomIntelligenceTrack;
  using PublicizedMessage = Frame;

  ROSCommunicationStrategy::ROSCommunicationStrategy(std::string const& sensor_model) :
      sensor_model_(sensor_model),
      message_publisher_(node_handle_.advertise<PublicizedMessage>(sensor_model, 1000))
  {
  }

  ROSCommunicationStrategy::~ROSCommunicationStrategy() noexcept
  {
  }

  void ROSCommunicationStrategy::openConnection(std::string const& publicizedTopic)
  {
    ROS_INFO("Connecting %s sensor", sensor_model_.c_str());
  }

  void ROSCommunicationStrategy::closeConnection()
  {
    ROS_INFO("Disconnecting %s sensor", sensor_model_.c_str());
  }

  void ROSCommunicationStrategy::sendMessage(MESSAGE&& message)
  {
    if(ros::ok())
    {

      Frame msg;

      msg.header.stamp = ros::Time::now();
      msg.header.seq++;

      msg.frame_id = message.frameID;
      msg.system_id = message.systemID;

      auto pixels = *message.getPixels();
      uint16_t number_of_pixels = pixels.size();
      msg.pixels.reserve(number_of_pixels);

      auto pixels_end = std::end(pixels);
      auto pixel_iterator = std::begin(pixels);
      for (pixel_iterator; pixel_iterator != pixels_end; ++pixel_iterator)
      {

        Pixel current_pixel;

        current_pixel.id = pixel_iterator->ID;

        auto tracks = *pixel_iterator->getTracks();
        uint16_t number_of_tracks = tracks.size();
        current_pixel.tracks.reserve(number_of_tracks);

        auto tracks_end = std::end(tracks);
        auto track_iterator = std::begin(tracks);
        for (track_iterator; track_iterator != tracks_end; ++track_iterator)
        {

          Track current_track;

          current_track.id = track_iterator->ID;
          current_track.confidence_level = track_iterator->confidenceLevel;
          current_track.intensity = track_iterator->intensity;
          current_track.acceleration = track_iterator->acceleration;
          current_track.distance = track_iterator->distance;
          current_track.speed = track_iterator->speed;

          current_pixel.tracks.push_back(current_track);
        }

        msg.pixels.push_back(current_pixel);
      }

      message_publisher_.publish(msg);

      ros::spinOnce();
    }
  }
}
