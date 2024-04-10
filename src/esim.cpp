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
#include <gazebo_dvs_plugin/esim.hpp>
////////////////////////////////////////////////////////////////////////////////
// Constructor
Esim::Esim()
{
  velocity.x = 0;
  velocity.y = 0;
  velocity.z = 0;
  angular_velocity.x = 0;
  angular_velocity.y = 0;
  angular_velocity.z = 0;
  last_time = ros::Time::now();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Esim::~Esim()
{
}
void Esim::simulateESIM(cv::Mat *last_image, cv::Mat *curr_image, std::vector<dvs_msgs::Event> *events, sensor_msgs::Imu &imu_msg, sensor_msgs::Image &msg_dep_img, ros::Time &current_time, ros::Time &last_time)
{
  *last_image += 1e-4;
  *curr_image += 1e-4;

  // convert the image to float

  last_image->convertTo(*last_image, CV_32F);
  curr_image->convertTo(*curr_image, CV_32F);

  // convert to log intensity
  cv::log(*last_image, *last_image);
  cv::log(*curr_image, *curr_image);
  // get the imu and depth messages

  assert(msg_dep_img.encoding == sensor_msgs::image_encodings::TYPE_32FC1);
  // convert the depth image from senso::Image to float
  assert(msg_dep_img.data.size() % sizeof(float) == 0);
  float *depth_image = reinterpret_cast<float *>(msg_dep_img.data.data());

  if (imu_msg.orientation_covariance[0] < 0)
  {
    return;
  }
  // calculate the timestamp between two consecutive frames
  ros::Time curr_time = imu_msg.header.stamp;

  float delta_t_ = curr_time.sec - this->last_time.sec + (curr_time.nsec - this->last_time.nsec) / 1e9;
  this->last_time = curr_time;

  this->velocity.x += imu_msg.linear_acceleration.x * delta_t_;
  this->velocity.y += imu_msg.linear_acceleration.y * delta_t_;
  this->velocity.z += imu_msg.linear_acceleration.z * delta_t_;
  this->angular_velocity.x = imu_msg.angular_velocity.x;
  this->angular_velocity.y = imu_msg.angular_velocity.y;
  this->angular_velocity.z = imu_msg.angular_velocity.z;

  double min_t_v, min_t_b;
  this->adaptiveSample(last_image, curr_image, depth_image, current_time, last_time, &min_t_v, &min_t_b);
  double lambda_b = 0.5, lambda_v = 0.5;
  // the sample interval between two events for events generation is 1 us.
  double t_sample_interval = std::max(this->MIN_TIME_INTERVAL, lambda_b * min_t_b + lambda_v * min_t_v);
  

  this->fillEvents(static_cast<int>(min_t_b*1e9), static_cast<int>(min_t_v*1e9), current_time, 1, events);
}

void Esim::egoVelocity(const float Z, const float u, const float v, float *B)
{
  // page 12 formula 4 for "ESIM: an Open Event Camera Simulator"
  *B = std::fabs(-1 / Z * this->velocity.x + u / Z * this->velocity.z + u * v * this->angular_velocity.x - (1 + u * u) * this->angular_velocity.y + v * this->angular_velocity.z) + std::fabs(-1 / Z * this->velocity.y + v / Z * this->velocity.z + (1 + v * v) * this->angular_velocity.x - u * v * this->angular_velocity.y - u * this->angular_velocity.z);
}

void Esim::lightChange(const float l1, const float l2, const float f_current_time, const float f_last_time, float *delta_l)
{
  // l1 and l2 are the logirithmic light intensities of the two frames
  *delta_l = (l2 - l1) / (f_current_time - f_last_time);
}

void Esim::adaptiveSample(cv::Mat *last_image, const cv::Mat *curr_image, const float *curr_dep_img_, const ros::Time &current_time, const ros::Time &last_time, double *min_t_v, double *min_t_b)
{
  float f_current_time = current_time.toSec();
  float f_last_time = last_time.toSec();

  cv::Mat temp_image = cv::Mat::zeros(last_image->rows, last_image->cols, CV_32F);

  // calculate the velocity and angular velocity for the camera ego movement
  temp_image.forEach<float>([&](float &pixel, const int *position) -> void
                            {
                                  float Z = curr_dep_img_[(position[0]*last_image->rows+ position[1])];
                                  this->egoVelocity(Z, position[0], position[1], &pixel); });
  // calculate the light change between the two frames
  last_image->forEach<float>([&](float &l1, const int *position) -> void
                             {
    float l2 = curr_image->at<uchar>(position[0], position[1]);
    this->lightChange(l1, l2, f_current_time, f_last_time, &l1); });
  double max;
  cv::Point min_loc, max_loc;
  // get the minimum value of the two
  cv::minMaxLoc(temp_image, min_t_v, &max, &min_loc, &max_loc);
  cv::minMaxLoc(*last_image, min_t_b, &max, &min_loc, &max_loc);
}

// void Esim::fillEvents(cv::Mat *mask, int polarity, std::vector<dvs_msgs::Event> *events)
void Esim::fillEvents(int x, int y, ros::Time ts, int p, std::vector<dvs_msgs::Event> *events)
{
  // findNonZero fails when there are no zeros
  dvs_msgs::Event event;
  event.x = x;
  event.y = y;
  event.ts = ts;
  event.polarity = p;
  events->push_back(event);
}