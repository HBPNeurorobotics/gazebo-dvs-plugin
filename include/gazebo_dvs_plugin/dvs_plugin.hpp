/**---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
 * This file is part of the Neurorobotics Platform software
 * Copyright (C) 2014,2015,2016,2017 Human Brain Project
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * ---LICENSE-END**/
/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * bla: Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef DVS_PLUGIN_HPP
#define DVS_PLUGIN_HPP

#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/DepthCameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/rendering/DepthCamera.hh>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <cmath>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <gazebo_dvs_plugin/esim.hpp>
using namespace std;
using namespace cv;

namespace gazebo
{
  class GAZEBO_VISIBLE DvsPlugin : public SensorPlugin
  {
  public:
    DvsPlugin();
    ~DvsPlugin();
    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

  protected:
    virtual void mainCallback(const unsigned char *_image,
                              unsigned int _width, unsigned int _height,
                              unsigned int _depth, const string &_format);

  protected:
    unsigned int width, height, depth;
    string format;

    sensors::CameraSensorPtr parentCameraSensor;
    rendering::CameraPtr camera;
    // interpolate start time and end time between frames
    ros::Time last_time_, current_time_;

    event::ConnectionPtr newFrameConnection;

    ros::NodeHandle node_handle_;
    ros::Publisher event_pub_;
    string namespace_;

    // for imu and depth data accquisition
    ros::Subscriber imu_sub_, dep_sub_;
    // depth image
    sensor_msgs::Image dep_img_;
    // store a sequence of imu messages for ESIM computing.
    sensor_msgs::Imu imu_msg_;

  private:
    Mat last_image;
    bool has_last_image,imu_cali_flag;
    float event_threshold;
    Esim esim;

  private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void processDelta(Mat *last_image, Mat *curr_image, vector<dvs_msgs::Event> *events);
    void fillEvents(Mat *diff, int polarity, vector<dvs_msgs::Event> *events);
    void publishEvents(vector<dvs_msgs::Event> *events);
    void depthCallback(const sensor_msgs::ImageConstPtr &msg);
  };
}
#endif
