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

#include "ros_phantom_intelligence/awl16_connection.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "AWL16");

  std::string device_location;

  ros::param::param<std::string>("~device_location", device_location, "0");
  int32_t const device_channel = std::stoi(device_location);

  using AWL16Connection = phantom_intelligence_driver::awl16::AWL16Connection;
  AWL16Connection sensor_connection(device_channel);

  sensor_connection.start();

  std::this_thread::sleep_for(std::chrono::seconds(10000));

  sensor_connection.terminateAndJoin();

  ros::shutdown();
  return 0;
}
