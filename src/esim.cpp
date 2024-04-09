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
 */
#ifdef _WIN32
// Ensure that Winsock2.h is included before Windows.h, which can get
// pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include <ros/ros.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <gazebo/common/Plugin.hh>

#include <sensor_msgs/Imu.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

// #include <gazebo_dvs_plugin/dvs_plugin.hpp>
#include <gazebo_dvs_plugin/esim.hpp>

void simulateESIM(cv::Mat *last_image, cv::Mat *curr_image, std::vector<sensor_msgs::Imu> &imu_msgs, std::vector<dvs_msgs::Event> *events, const float *curr_dep_img_, ros::Time &current_time, ros::Time &last_time)
{ // get the imu messages
  if (imu_msgs.size() > 0)
  {
    for (auto imu_msg : imu_msgs)
    {
      if (imu_msg.orientation_covariance[0] < 0)
      {
        continue;
      }

      // ros::Time stamp = imu_msg.header.stamp;
      // float delta_t_ = stamp.sec - this->stamp_last_imu_.sec + (stamp.nsec - this->stamp_last_imu_.nsec) / 1e9;
      geometry_msgs::Quaternion orientation;
      geometry_msgs::Vector3 angular_velocity, linear_acceleration;
      orientation = imu_msg.orientation;
      angular_velocity = imu_msg.angular_velocity;
      linear_acceleration = imu_msg.linear_acceleration;
      this->velocity_x += linear_acceleration.x * delta_t_;
      this->velocity_y += linear_acceleration.y * delta_t_;
      this->velocity_z += linear_acceleration.z * delta_t_;
      this->angular_velocity_x = angular_velocity.x;
      this->angular_velocity_y = angular_velocity.y;
      this->angular_velocity_z = angular_velocity.z;

      // transform the Quaternion orientation to roll, pitch, yaw
      tf::Quaternion q(
          orientation.x,
          orientation.y,
          orientation.z,
          orientation.w);
      tf::Matrix3x3 m(q);
      float roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      float min_t_v, min_t_b;
      adaptiveSample(last_image, curr_image, curr_dep_img_, current_time, last_time, &min_t_v, &min_t_b);
      float lambda_b = 0.5, lambda_v = 0.5;
      float t_sample_interval = lambda_b * min_t_b + lambda_v * min_t_v;
    }
  }
}

void egoVelocity(const float Z, const float u, const float v, const geometry_msgs::Vector3 &velocity, const geometry_msgs::Vector3 &angular_velocity, float *B)
{
  // page 12 formula 4 for "ESIM: an Open Event Camera Simulator"
  *B = std::fabs(-1 / Z * velocity.x + u / Z * velocity.z + u * v * angular_velocity.x - (1 + u * u) * angular_velocity.y + v * angular_velocity.z) + std::fabs(-1 / Z * velocity.y + v / Z * velocity.z + (1 + v * v) * angular_velocity.x - u * v * angular_velocity.y - u * angular_velocity.z);
}

void lightChange(const float l1, const float l2, const float f_current_time, const float f_last_time, float *delta_l)
{
  // l1 and l2 are the logirithmic light intensities of the two frames
  *delta_l = (l2 - l1) / (f_current_time - f_last_time);
}

void adaptiveSample(cv::Mat *last_image, const cv::Mat *curr_image, const float *curr_dep_img_, const ros::Time &current_time, const ros::Time &last_time, const geometry_msgs::Vector3 &velocity, const geometry_msgs::Vector3 &angular_velocity, float *min_t_v, float *min_t_b)
{
  float f_current_time = current_time.toSec();
  float f_last_time = last_time.toSec();

  cv::Mat temp_image = cv::Mat::zeros(last_image->rows, last_image->cols, CV_32F);

  // calculate the velocity and angular velocity for the camera ego movement
  temp_image.forEach<float>([&](float &pixel, const int *position) -> void
                            {
                                  float Z = curr_dep_img_[(position[0]*last_image->rows+ position[1])];
                                  egoVelocity(Z, position[0], position[1], velocity, angular_velocity, &pixel); });
  // calculate the light change between the two frames
  last_image.forEach<float>([&](float &l1, const int *position) -> void
                            {
    float l2 = curr_image->at<uchar>(position[0], position[1]);
    lightChange(l1, l2, f_current_time, f_last_time, &l1); });
  float max;
  cv::Point min_loc, max_loc;
  // get the minimum value of the two
  cv::minMaxLoc(temp_image, &min_t_v, &max, &min_loc, &max_loc);
  cv::minMaxLoc(last_image, &min_t_b, &max, &min_loc, &max_loc);
}

void fillEvents(cv::Mat *mask, int polarity, std::vector<dvs_msgs::Event> *events)
{
  // findNonZero fails when there are no zeros
  // TODO is there a better workaround then iterating the binary image twice?
  if (cv::countNonZero(*mask) != 0)
  {
    std::vector<cv::Point> locs;
    cv::findNonZero(*mask, locs);

    for (int i = 0; i < locs.size(); i++)
    {
      dvs_msgs::Event event;
      event.x = locs[i].x;
      event.y = locs[i].y;
      event.ts = ros::Time::now();
      event.polarity = polarity;
      events->push_back(event);
    }
  }
}