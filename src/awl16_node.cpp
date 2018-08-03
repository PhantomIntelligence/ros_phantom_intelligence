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

#include "ros/ros.h"
#include "phantom_intelligence/SpiritFrame.h"
#include "ros_phantom_intelligence/awl_connection.h"

#include <sstream>

/*
 * TODO: make a class ROSCommunicationStrategy: public ServerCommunicationStrategy<DataFlow::Frame>
 *  ctor -> takes device location, model, etc...
 *  connect -> ros::init(...) & cie; ROS_INFO(sensor connected w/ metadata); ...
 *  disconnect -> ROS_INFO(sensor disconnected, etc;...
 *  sendMessage -> if(ros::ok()) { receives DataFlow::Frame => SpiritFrame.msg; publish(msg); ros::spinOnce, loop_rate.sleep() };
 *
 * TODO: have the awl_connection class use the ROSCommunicationStrategy
 * TODO: fix this node class so it builds
 *
 */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "awl16");
  ros::NodeHandle n;

  using PublicisedMessage = phantom_intelligence::SpiritFrame;

  ros::Publisher awl16_pub = n.advertise<PublicisedMessage>("awl16", 1000);
  ros::Rate loop_rate(10);

  //  phantom_intelligence_driver::awl::AWLConnection awl_connection(nullptr);
  phantom_intelligence_driver::awl::AWLConnection awl_connection;
  awl_connection.connect();
  awl_connection.disconnect();
  int count = 0;
  while (ros::ok())
  {
    PublicisedMessage msg;

    std::stringstream ss;
    ss << "msg: " << count;
    msg.header.frame_id = ss.str();

    ROS_INFO("%s", msg.header.frame_id.c_str());

    awl16_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
