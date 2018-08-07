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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "AWL16");

  std::string device_location;

  ros::param::param<std::string>("~device_location", device_location, "0");

  using AWL16Connection = phantom_intelligence_driver::awl16::AWL16Connection;
  AWL16Connection sensor_connection(device_location);

  sensor_connection.connect();

  std::this_thread::sleep_for(std::chrono::seconds(10));

  sensor_connection.disconnect();

  return 0;
}
