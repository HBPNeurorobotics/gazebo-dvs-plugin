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

#include <string>
#include <vector>
#include <ros/ros.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <gazebo/util/system.hh>


#include <sensor_msgs/Imu.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <opencv2/opencv.hpp>
// #include <gazebo_dvs_plugin/esim.hpp>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace cv;

namespace gazebo
{
  class GAZEBO_VISIBLE DvsPlugin : public SensorPlugin
  {
    public: DvsPlugin();

    public: ~DvsPlugin();

    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    protected: virtual void mainCallback(const unsigned char *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const string &_format);

    protected: unsigned int width, height, depth;
    protected: string format;

    protected: sensors::CameraSensorPtr parentCameraSensor;
    protected: rendering::CameraPtr camera;
    // interpolate start time and end time between frames
    protected: ros::Time last_time_, current_time_;

    // depth camera sensor and depth image
    protected: const float* curr_dep_img_;

    private: event::ConnectionPtr cameraUpdateConnection;

    protected: ros::NodeHandle node_handle_;
    protected: ros::Publisher event_pub_;
    protected: string namespace_;
    
    // for imu data accquisition
    protected: ros::Subscriber imu_sub_, dep_sub_;
    protected: ros::Publisher imu_pub_, dep_pub_;
    // protected: sensor_msgs::Imu latest_imu_msg_;
    // store a sequence of imu messages for ESIM computing.
    protected: std::vector<sensor_msgs::Imu> imu_msgs_;

    private: Mat last_image;
    private: bool has_last_image;
    private: float event_threshold;
    

    private:
      void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

    private: void processDelta(Mat *last_image, Mat *curr_image, vector<dvs_msgs::Event> *events);
    private: void fillEvents(Mat *diff, int polarity, vector<dvs_msgs::Event> *events);
    private: void publishEvents(vector<dvs_msgs::Event> *events);
    private: 
      void depthCallback();
      void cameraCallback();
      // private: void EsimProcess(cv::Mat *last_image, cv::Mat *curr_image, std::vector<sensor_msgs::Imu> &imu_msgs, std::vector<dvs_msgs::Event> *events, const float * depthImage, const float current_time, const float last_time);
  };
}
#endif
