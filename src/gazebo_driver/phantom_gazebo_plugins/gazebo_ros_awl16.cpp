//
// Created by phil on 3/5/19.
//

/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: Ros Laser controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 */

#include <algorithm>
#include <string>
#include <assert.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>
#include "phantom_intelligence/PhantomIntelligenceFrame.h"
#include "phantom_intelligence/PhantomIntelligenceRawData.h"
#include "phantom_intelligence/PhantomIntelligenceFrame.h"
#include "phantom_intelligence/PhantomIntelligencePixel.h"
#include "phantom_intelligence/PhantomIntelligenceTrack.h"

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "gazebo_ros_awl16.h"

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosAWL16)


////////////////////////////////////////////////////////////////////////////////
// Constructor
        GazeboRosAWL16::GazeboRosAWL16()
        {
            this->seed = 0;
        }

////////////////////////////////////////////////////////////////////////////////
// Destructor
        GazeboRosAWL16::~GazeboRosAWL16()
        {
            this->rosnode_->shutdown();
            delete this->rosnode_;
        }

////////////////////////////////////////////////////////////////////////////////
// Load the controller
        void GazeboRosAWL16::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
        {
            // load plugin
            RayPlugin::Load(_parent, this->sdf);
            // Get the world name.
# if GAZEBO_MAJOR_VERSION >= 7
            std::string worldName = _parent->WorldName();
# else
            std::string worldName = _parent->GetWorldName();
# endif

            this->world_ = physics::get_world(worldName);
            // save pointers
            this->sdf = _sdf;

            GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
            this->parent_ray_sensor_ =
                    dynamic_pointer_cast<sensors::RaySensor>(_parent);

            if (!this->parent_ray_sensor_)
            gzthrow("GazeboRosAWL16 controller requires a Ray Sensor as its parent");

            this->robot_namespace_ =  GetRobotNamespace(_parent, _sdf, "Laser");

            if (!this->sdf->HasElement("frameName"))
            {
                ROS_INFO("Laser plugin missing <frameName>, defaults to /world");
                this->frame_name_ = "/world";
            }
            else
                this->frame_name_ = this->sdf->Get<std::string>("frameName");


            if (!this->sdf->HasElement("topicName"))
            {
                ROS_INFO("Laser plugin missing <topicName>, defaults to /world");
                this->topic_name_ = "/world";
            }
            else
                this->topic_name_ = this->sdf->Get<std::string>("topicName");

            this->laser_connect_count_ = 0;

            // Make sure the ROS node for Gazebo has already been initialized
            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }

            ROS_INFO ( "Starting Laser Plugin (ns = %s)!", this->robot_namespace_.c_str() );
            // ros callback queue for processing subscription
            this->deferred_load_thread_ = boost::thread(
                    boost::bind(&GazeboRosAWL16::LoadThread, this));

        }

////////////////////////////////////////////////////////////////////////////////
// Load the controller
        void GazeboRosAWL16::LoadThread()
        {
            using ROSFrame = phantom_intelligence::PhantomIntelligenceFrame;
            using ROSPixel = phantom_intelligence::PhantomIntelligencePixel;
            using ROSTrack = phantom_intelligence::PhantomIntelligenceTrack;
            this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
            this->gazebo_node_->Init(this->world_name_);

            this->pmq.startServiceThread();

            this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

            this->tf_prefix_ = tf::getPrefixParam(*this->rosnode_);
            if(this->tf_prefix_.empty()) {
                this->tf_prefix_ = this->robot_namespace_;
                boost::trim_right_if(this->tf_prefix_,boost::is_any_of("/"));
            }
            ROS_INFO("Laser Plugin (ns = %s)  <tf_prefix_>, set to \"%s\"",
                     this->robot_namespace_.c_str(), this->tf_prefix_.c_str());

            // resolve tf prefix
            this->frame_name_ = tf::resolve(this->tf_prefix_, this->frame_name_);

            if (this->topic_name_ != "")
            {
                ros::AdvertiseOptions ao =
                        ros::AdvertiseOptions::create<ROSFrame>(
                                this->topic_name_, 1,
                                boost::bind(&GazeboRosAWL16::LaserConnect, this),
                                boost::bind(&GazeboRosAWL16::LaserDisconnect, this),
                                ros::VoidPtr(), NULL);
                this->pub_ = this->rosnode_->advertise(ao);
                this->pub_queue_ = this->pmq.addPub<ROSFrame>();
            }

            // Initialize the controller

            // sensor generation off by default
            this->parent_ray_sensor_->SetActive(false);
        }

////////////////////////////////////////////////////////////////////////////////
// Increment count
        void GazeboRosAWL16::LaserConnect()
        {
            this->laser_connect_count_++;
            if (this->laser_connect_count_ == 1)
                this->laser_scan_sub_ =
# if GAZEBO_MAJOR_VERSION >= 7
                        this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
# else
                                this->gazebo_node_->Subscribe(this->parent_ray_sensor_->GetTopic(),
# endif
                                                      &GazeboRosAWL16::OnScan, this);
        }

////////////////////////////////////////////////////////////////////////////////
// Decrement count
        void GazeboRosAWL16::LaserDisconnect()
        {
            this->laser_connect_count_--;
            if (this->laser_connect_count_ == 0)
                this->laser_scan_sub_.reset();
        }


// Convert new Gazebo message to ROS message and publish it
void GazeboRosAWL16::OnScan(ConstLaserScanStampedPtr &_msg)
{

    int awl16_num_pixel = 8;
    int awl16_num_pixel_row = 2;
    int awl16_detection_per_pixel = 5;
    int awl16_num_vertical_rays = 10;
  // We got a new message from the Gazebo sensor.  Stuff a
  // corresponding ROS message and publish it.
    ROSFrame frame_msg;
    // What to do when callback
    std::vector<ROSPixel> pixel_list_first_row;
    std::vector<ROSPixel> pixel_list_second_row;
    for (int i = 0; i < awl16_num_pixel; ++i)
    {
        ROSPixel pixel_msg_first_row;
        ROSPixel pixel_msg_second_row;
        pixel_msg_first_row.id = static_cast<unsigned short>(i);
        pixel_msg_second_row.id = static_cast<unsigned short>(i+8);
        std::vector<ROSTrack> track_list_first_row;
        std::vector<ROSTrack> track_list_second_row;
        for (int j=0; j < awl16_detection_per_pixel; ++j)
        {
            ROSTrack track_msg_first_row;
            ROSTrack track_msg_second_row;
            track_msg_first_row.id = static_cast<unsigned short>(j);
            track_msg_second_row.id = static_cast<unsigned short>(j);
            double temp_min_distance_first_row  = _msg->scan().ranges().Get(i*awl16_detection_per_pixel+j); //first value of the 4 vertical rays that simulate the vertical part of thr foi
            double temp_min_distance_second_row  = _msg->scan().ranges().Get(i*awl16_detection_per_pixel+j+(6*awl16_detection_per_pixel*awl16_num_pixel)); //first value of the last 4 vertical rays that simulate the vertical part of thr foi
            for (int k=0; k < 4; ++k)
            {
                double distance_first_row = _msg->scan().ranges().Get(i*awl16_detection_per_pixel+j+(k*awl16_detection_per_pixel*awl16_num_pixel));
                double distance_second_row  = _msg->scan().ranges().Get(i*awl16_detection_per_pixel+j+((k+6)*awl16_detection_per_pixel*awl16_num_pixel));
                if (temp_min_distance_first_row > distance_first_row)
                {
                    temp_min_distance_first_row = distance_first_row; //we keep the closest object detected on all the vertical rays for each portions of the pixel.
                }
                if (temp_min_distance_second_row > distance_second_row)
                {
                    temp_min_distance_second_row = distance_second_row; //we keep the closest object detected on all the vertical rays for each portions of the pixel.
                }

            }
            track_msg_first_row.distance = (temp_min_distance_first_row);
            track_msg_second_row.distance = (temp_min_distance_second_row);
            track_list_first_row.push_back(track_msg_first_row);
            track_list_second_row.push_back(track_msg_second_row);
        }
        pixel_msg_first_row.tracks = track_list_first_row;
        pixel_msg_second_row.tracks = track_list_second_row;
        pixel_list_first_row.push_back(pixel_msg_first_row);
        pixel_list_second_row.push_back(pixel_msg_second_row);
    }
    pixel_list_first_row.insert(pixel_list_first_row.end(), pixel_list_second_row.begin(), pixel_list_second_row.end());
    frame_msg.pixels = pixel_list_first_row;

  this->pub_queue_->push(frame_msg, this->pub_);
}
}