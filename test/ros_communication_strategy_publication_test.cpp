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
  INCLUDING, BUT NOT LIMITED TO, THE "encoding/binary"IMPLIED WARRANTIES
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

#include <future>
#include <random>
#include <limits>
#include <gtest/gtest.h>

#include <sensor-gateway/common/data-structure/spirit/SpiritStructures.h>

#include "ros_phantom_intelligence/ros_communication_strategy.h"

static size_t const NUMBER_OF_RAW_DATA_CHANNELS = 1;

static size_t const NUMBER_OF_SAMPLES_PER_CHANNEL = 10;

using FakeRawDataDefinition = typename
Sensor::RawDataDefinition<int16_t, NUMBER_OF_RAW_DATA_CHANNELS, NUMBER_OF_SAMPLES_PER_CHANNEL>;
using FakeDataStructures = typename Sensor::Spirit::Structures<FakeRawDataDefinition>;

using SensorFrame = DataFlow::Frame;
using SensorPixel = DataFlow::Pixel;
using SensorTrack = DataFlow::Track;

using ROSRawData = phantom_intelligence::PhantomIntelligenceRawData;

using ROSFrame = phantom_intelligence::PhantomIntelligenceFrame;
using ROSPixel = phantom_intelligence::PhantomIntelligencePixel;
using ROSTrack = phantom_intelligence::PhantomIntelligenceTrack;

class ROSCommunicationStrategyPublicationTest : public ::testing::Test
{
protected:
  std::string const TEST_TOPIC_NAME = "UNDEFINED";

  ROSCommunicationStrategyPublicationTest()
  {
  }

  ~ROSCommunicationStrategyPublicationTest()
  {
  }

  SensorFrame createRandomFrame() const noexcept
  {
    SensorFrame frame;
    std::default_random_engine random_engine(std::random_device{}());

    std::uniform_int_distribution <DataFlow::FrameID> id_distribution(
        std::numeric_limits<DataFlow::FrameID>::lowest(),
        std::numeric_limits<DataFlow::FrameID>::max());

    std::uniform_int_distribution <DataFlow::SystemID> system_id_distribution(
        std::numeric_limits<DataFlow::SystemID>::lowest(),
        std::numeric_limits<DataFlow::SystemID>::max());

    frame.frameID = id_distribution(random_engine);
    frame.systemID = system_id_distribution(random_engine);
    std::bernoulli_distribution populate_pixel(0.5); // True 50% of time
    std::bernoulli_distribution populate_track(0.1); // True 10% of time
    auto max_number_of_pixel = Sensor::AWL::_16::NUMBER_OF_PIXELS_IN_FRAME;
    auto max_number_of_track = Sensor::AWL::_16::NUMBER_OF_TRACKS_IN_PIXEL;

    for (auto pixel_id = 0u; pixel_id < max_number_of_pixel; ++pixel_id)
    {
      if(populate_pixel(random_engine))
      {
        for (auto track_iterator = 0; track_iterator < max_number_of_track; ++track_iterator)
        {
          if(populate_track(random_engine))
          {
            frame.addTrackToPixelWithID(pixel_id, std::move(createRandomSensorTrack(&random_engine)));
          }
        }
      }
    }
    return frame;
  }

  ROSFrame createExpectedMessageFromSensorFrame(SensorFrame sensor_frame)
  {
    ROSFrame expected_frame;

    expected_frame.header.system_id = sensor_frame.systemID;
    expected_frame.id = sensor_frame.frameID;

    auto pixels = *sensor_frame.getPixels();
    uint16_t number_of_pixels = pixels.size();
    expected_frame.pixels.reserve(number_of_pixels);

    for (auto pixel_iterator = 0u; pixel_iterator < number_of_pixels; ++pixel_iterator)
    {

      ROSPixel current_pixel;

      current_pixel.id = pixel_iterator;

      auto tracks = *pixels[pixel_iterator].getTracks();

      auto number_of_tracks = pixels[pixel_iterator].getCurrentNumberOfTracksInPixel();

      if(number_of_tracks > 0)
      {

        current_pixel.tracks.reserve(number_of_tracks);

        for (auto track_iterator = 0u; track_iterator < number_of_tracks; ++track_iterator)
        {

          ROSTrack current_track;
          auto track_from_sensor_frame = &tracks[track_iterator];

          current_track.id = track_from_sensor_frame->ID;
          current_track.confidence_level = track_from_sensor_frame->confidenceLevel;
          current_track.intensity = track_from_sensor_frame->intensity;
          current_track.acceleration = track_from_sensor_frame->acceleration;
          current_track.distance = track_from_sensor_frame->distance;
          current_track.speed = track_from_sensor_frame->speed;

          current_pixel.tracks.push_back(current_track);
        }
      }

      expected_frame.pixels.push_back(current_pixel);
    }

    return expected_frame;
  }

  FakeDataStructures::RawData createRandomRawData() const noexcept
  {
    FakeDataStructures::RawData raw_data;
    std::default_random_engine random_engine(std::random_device{}());

    std::uniform_int_distribution <FakeRawDataDefinition::ValueType> raw_data_value_distribution(
        std::numeric_limits<FakeRawDataDefinition::ValueType>::lowest(),
        std::numeric_limits<FakeRawDataDefinition::ValueType>::max());

    auto max_number_of_values = FakeRawDataDefinition::SIZE;

    for (auto value_id = 0u; value_id < max_number_of_values; ++value_id)
    {
      raw_data.content[value_id] = raw_data_value_distribution(random_engine);
    }
    return raw_data;
  }

  ROSRawData createExpectedRawDataFromSpiritRawData(FakeDataStructures::RawData sensor_raw_data)
  {
    ROSRawData expected_raw_data;

    auto raw_data_values = sensor_raw_data.content;
    expected_raw_data.data.reserve(raw_data_values.size());

    for (auto raw_data_value_iterator = 0u; raw_data_value_iterator < raw_data_values.size(); ++raw_data_value_iterator)
    {
      expected_raw_data.data[raw_data_value_iterator] = raw_data_values[raw_data_value_iterator];
    }

    return expected_raw_data;
  }

private:

  static SensorTrack createRandomSensorTrack(std::default_random_engine* random_engine)
  {
    std::uniform_int_distribution <DataFlow::TrackID> track_id_distribution(
        std::numeric_limits<DataFlow::TrackID>::lowest(),
        std::numeric_limits<DataFlow::TrackID>::max());

    std::uniform_int_distribution <DataFlow::ConfidenceLevel> confidence_level_distribution(
        std::numeric_limits<DataFlow::ConfidenceLevel>::lowest(),
        std::numeric_limits<DataFlow::ConfidenceLevel>::max());

    std::uniform_int_distribution <DataFlow::Distance> distance_distribution(
        std::numeric_limits<DataFlow::Distance>::lowest(),
        std::numeric_limits<DataFlow::Distance>::max());

    std::uniform_int_distribution <DataFlow::Acceleration> acceleration_distribution(
        std::numeric_limits<DataFlow::Acceleration>::lowest(),
        std::numeric_limits<DataFlow::Acceleration>::max());

    std::uniform_int_distribution <DataFlow::Intensity> intensity_distribution(
        std::numeric_limits<DataFlow::Intensity>::lowest(),
        std::numeric_limits<DataFlow::Intensity>::max());

    std::uniform_int_distribution <DataFlow::Speed> speed_distribution(
        std::numeric_limits<DataFlow::Speed>::lowest(),
        std::numeric_limits<DataFlow::Speed>::max());

    SensorTrack track(track_id_distribution(*random_engine),
                      confidence_level_distribution(*random_engine),
                      intensity_distribution(*random_engine),
                      acceleration_distribution(*random_engine),
                      distance_distribution(*random_engine),
                      speed_distribution(*random_engine));

    return track;
  }
};

struct ROSFrameMessageCatcher
{
  ROSFrameMessageCatcher()
  {
  }

  void captureROSFrame(ROSFrame const& frame)
  {
    caught_frame_.set_value(frame);
  }

  ROSFrame fetchCaughtROSFrame()
  {
    auto future_frame = caught_frame_.get_future();
    future_frame.wait();
    return future_frame.get();
  }

  std::promise<ROSFrame> caught_frame_;
};

static void spinUntilTestIsOver(std::atomic<bool>* test_is_over)
{
  while (!test_is_over->load())
  {
    ros::spinOnce();
  }
}

TEST_F(ROSCommunicationStrategyPublicationTest,
    given_aFrameFromTheGateway_when_sendMessage_then_willPublishACorrectlyFormattedROSFrame)
{
  std::atomic<bool> test_is_over(false);
  auto fake_sensor_frame = createRandomFrame();
  auto fake_sensor_frame_copy = SensorFrame(fake_sensor_frame);

  phantom_intelligence_driver::ros_communication::ROSCommunicationStrategy<FakeDataStructures>
      ros_communication_strategy(TEST_TOPIC_NAME);

  ROSFrameMessageCatcher frame_message_catcher;
  ros::NodeHandle node_handle;
  auto message_topic_name = ros_communication_strategy.fetchMessageTopicName(TEST_TOPIC_NAME);
  ros::Subscriber subscriber = node_handle.subscribe(message_topic_name,
                                                     1000,
                                                     &ROSFrameMessageCatcher::captureROSFrame,
                                                     &frame_message_catcher);

  JoinableThread spinning_thread(spinUntilTestIsOver, &test_is_over);

  ros_communication_strategy.sendMessage(std::move(fake_sensor_frame));

  auto expected_message = createExpectedMessageFromSensorFrame(fake_sensor_frame_copy);

  ROSFrame actual_message = frame_message_catcher.fetchCaughtROSFrame();

  ASSERT_EQ(expected_message.id, actual_message.id);
  ASSERT_EQ(expected_message.header.system_id, actual_message.header.system_id);

  auto expected_pixels = expected_message.pixels;
  auto actual_pixels = actual_message.pixels;

  ASSERT_EQ(expected_pixels.size(), actual_pixels.size());

  for (auto pixel_iterator = 0; pixel_iterator < expected_pixels.size(); ++pixel_iterator)
  {
    auto expected_pixel = expected_pixels[pixel_iterator];
    auto actual_pixel = actual_pixels[pixel_iterator];

    ASSERT_EQ(expected_pixel .id, actual_pixel.id);

    auto expected_tracks = expected_pixel.tracks;

    auto actual_tracks = actual_pixel.tracks;

    ASSERT_EQ(expected_tracks.size(), actual_tracks.size());

    for (auto track_iterator = 0; track_iterator < expected_tracks.size(); ++track_iterator)
    {
      auto expected_track = expected_tracks[track_iterator];

      auto actual_track = actual_tracks[track_iterator];

      ASSERT_EQ(expected_track.id, actual_track.id);
      ASSERT_EQ(expected_track.confidence_level, actual_track.confidence_level);
      ASSERT_EQ(expected_track.intensity, actual_track.intensity);
      ASSERT_EQ(expected_track.acceleration, actual_track.acceleration);
      ASSERT_EQ(expected_track.distance, actual_track.distance);
      ASSERT_EQ(expected_track.speed, actual_track.speed);
    }
  }

  test_is_over.store(true);
  spinning_thread.join();
}

struct ROSRawDataCatcher
{
  ROSRawDataCatcher()
  {
  }

  void captureROSRawData(ROSRawData const& raw_data)
  {
    caught_raw_data_.set_value(raw_data);
  }

  ROSRawData fetchCaughtROSRawData()
  {
    auto future_raw_data = caught_raw_data_.get_future();
    future_raw_data.wait();
    return future_raw_data.get();
  }

  std::promise<ROSRawData> caught_raw_data_;
};

TEST_F(ROSCommunicationStrategyPublicationTest,
    given_rawDataFromTheGateway_when_sendRawData_then_willPublishACorrectlyFormattedROSRawData)
{
  std::atomic<bool> test_is_over(false);
  auto fake_raw_data = createRandomRawData();
  auto fake_raw_data_copy = FakeDataStructures::RawData(fake_raw_data);

  phantom_intelligence_driver::ros_communication::ROSCommunicationStrategy<FakeDataStructures>
      ros_communication_strategy(TEST_TOPIC_NAME);

  ROSRawDataCatcher raw_data_catcher;
  ros::NodeHandle node_handle;
  auto raw_data_topic_name = ros_communication_strategy.fetchRawDataTopicName(TEST_TOPIC_NAME);
  ros::Subscriber subscriber = node_handle.subscribe(raw_data_topic_name,
                                                     1000,
                                                     &ROSRawDataCatcher::captureROSRawData,
                                                     &raw_data_catcher);

  JoinableThread spinning_thread(spinUntilTestIsOver, &test_is_over);

  ros_communication_strategy.sendRawData(std::move(fake_raw_data));

  auto expected_raw_data = createExpectedRawDataFromSpiritRawData(fake_raw_data_copy);

  ROSRawData actual_raw_data = raw_data_catcher.fetchCaughtROSRawData();

  ASSERT_EQ(expected_raw_data.data.size(), actual_raw_data.data.size());

  for (auto raw_data_value_iterator = 0u; raw_data_value_iterator < expected_raw_data.data.size(); ++raw_data_value_iterator)
  {
    ASSERT_EQ(expected_raw_data.data[raw_data_value_iterator],
                actual_raw_data.data[raw_data_value_iterator]);
  }

  test_is_over.store(true);
  spinning_thread.join();
}

