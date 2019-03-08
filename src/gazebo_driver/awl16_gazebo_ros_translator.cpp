//
// Created by phil on 3/4/19.
//
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include "phantom_intelligence/PhantomIntelligenceHeader.h"
#include "phantom_intelligence/PhantomIntelligenceRawData.h"
#include "phantom_intelligence/PhantomIntelligenceFrame.h"
#include "phantom_intelligence/PhantomIntelligencePixel.h"
#include "phantom_intelligence/PhantomIntelligenceTrack.h"
// terminal_1:
// roscore
// terminal_2:
// rosrun gazebo_ros gazebo
// spawn romanoff model
//terminal_3
//cd ~/catkin_ws/devel/lib/phantom_intelligence
//./gazebo_awl16_node

ros::Publisher pub;
using ROSFrame = phantom_intelligence::PhantomIntelligenceFrame;
using ROSPixel = phantom_intelligence::PhantomIntelligencePixel;
using ROSTrack = phantom_intelligence::PhantomIntelligenceTrack;
int awl16_num_pixel = 8;
int awl16_num_pixel_row = 2;
int awl16_detection_per_pixel = 5;
int awl16_num_vertical_rays = 10;

// Gazebo LaserScan callback function
void scanCB(ConstLaserScanStampedPtr &_msg){
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
                track_msg_first_row.distance = static_cast<unsigned short>(temp_min_distance_first_row);
                track_msg_second_row.distance = static_cast<unsigned short>(temp_min_distance_second_row);
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
    pub.publish(frame_msg);

}

int main(int _argc, char **_argv){

    // Load Gazebo & ROS
    gazebo::client::setup(_argc, _argv);
    ros::init(_argc, _argv, "awl16_gazebo");

    // Create Gazebo node and init
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Create ROS node and init
    ros::NodeHandle n;
    pub = n.advertise<ROSFrame>("awl16", 1000);

    // Listen to Gazebo contacts topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/Romanoff/body/sensor1/scan", scanCB);


    // Busy wait loop...replace with your own code as needed.
    // Busy wait loop...replace with your own code as needed.
    while (ros::ok())
    {
        gazebo::common::Time::MSleep(20);

        // Spin ROS (needed for publisher) // (nope its actually for subscribers-calling callbacks ;-) )
        ros::spinOnce();


        // Mayke sure to shut everything down.

    }
    gazebo::client::shutdown();
}