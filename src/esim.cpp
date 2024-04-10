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
Esim::Esim() : event_threshold(10.0)
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

  float f_time_interval = current_time.toSec() - last_time.toSec();
  // convert the image to float

  last_image->convertTo(*last_image, CV_32F);
  curr_image->convertTo(*curr_image, CV_32F);

  // get the imu and depth messages

  // cv::log(*last_image, *last_image);
  // cv::log(*curr_image, *curr_image);
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

  float min_t_v, min_t_b;
  this->adaptiveSample(last_image, curr_image, depth_image, f_time_interval, &min_t_v, &min_t_b);
  float lambda_b = 0.5, lambda_v = 0.5;
  // the sample interval between two events for events generation is 1 us.
  float t_sample_interval = std::max(MIN_TIME_INTERVAL, lambda_b * min_t_b + lambda_v * min_t_v);
  // caculate the slope matrix between the two frames
  cv::Mat slope = cv::Mat::zeros(last_image->rows, last_image->cols, CV_32F);
  slope.forEach<float>([&](float &pixel, const int *position) -> void
                       { this->lightChange(last_image->at<float>(position[0], position[1]),
                                           curr_image->at<float>(position[0], position[1]),
                                           f_time_interval,
                                           &pixel); });
  cv::Mat last_image_, curr_image_;
  // convert to log intensity to store the last time after event was created
  // cv::log(*last_image, last_image_);
  last_image_ = last_image->clone();
  // use linear interpolation between the two frames.
  // add 1 means this function should be executed at least once.
  // for (uint64_t iter_num = 0; iter_num < static_cast<uint64_t>(f_time_interval / t_sample_interval) + 1; iter_num++)
  // {
    // convert to log intensity to store the current image during the interpolation
    // cv::log(*last_image + slope * t_sample_interval, curr_image_);
    curr_image_ = *last_image + slope * f_time_interval;
    // this->processDelta(&last_image_, &curr_image_, this->event_threshold, events);
    this->processDelta(last_image, curr_image, this->event_threshold, events);
  // }
  // for debug only
  // this->_debug_fillEvents(static_cast<int>(f_time_interval * 1e9), static_cast<int>(t_sample_interval * 1e9), current_time, 1, events);
}

// last_image : the last time after event was created
// curr_image : the current image
// this function will change the last_image in accordance with events activation after the event was created
void Esim::processDelta(cv::Mat *last_image, const cv::Mat *curr_image, const float event_threshold, std::vector<dvs_msgs::Event> *events)
{
  if (curr_image->size() == last_image->size())
  {
    cv::Mat pos_diff = *curr_image - *last_image;
    cv::Mat neg_diff = *last_image - *curr_image;

    cv::Mat pos_mask;
    cv::Mat neg_mask;

    cv::threshold(pos_diff, pos_mask, event_threshold, 255, cv::THRESH_BINARY);
    cv::threshold(neg_diff, neg_mask, event_threshold, 255, cv::THRESH_BINARY);

    // after event was created, these place where have yet activated should be set as the current value, instead of the initiated value.
    *last_image += pos_mask & pos_diff;
    *last_image -= neg_mask & neg_diff;

    this->fillEvents(&pos_mask, 0, events);
    this->fillEvents(&neg_mask, 1, events);
  }
  else
  {
    std::cout << "Unexpected change in image size (" << last_image->size() << " -> " << curr_image->size() << "). Publishing no events for this frame change." << std::endl;
  }
}
void Esim::egoVelocity(const float Z, const float u, const float v, float *B)
{
  // page 12 formula 4 for "ESIM: an Open Event Camera Simulator"
  *B = std::fabs(-1 / Z * this->velocity.x + u / Z * this->velocity.z + u * v * this->angular_velocity.x - (1 + u * u) * this->angular_velocity.y + v * this->angular_velocity.z) + std::fabs(-1 / Z * this->velocity.y + v / Z * this->velocity.z + (1 + v * v) * this->angular_velocity.x - u * v * this->angular_velocity.y - u * this->angular_velocity.z);
}

void Esim::lightChange(const float last_pixel, const float curr_pixel, const float f_time_interval, float *delta_pixel)
{
  // l1 and l2 are the logirithmic light intensities of the two frames
  *delta_pixel = (last_pixel - curr_pixel) / f_time_interval;
}

void Esim::adaptiveSample(cv::Mat *last_image, const cv::Mat *curr_image, const float *curr_dep_img_, const float f_time_interval, float *min_t_v, float *min_t_b)
{
  cv::Mat temp_image = cv::Mat::zeros(last_image->rows, last_image->cols, CV_32F);

  // calculate the velocity and angular velocity for the camera ego movement
  temp_image.forEach<float>([&](float &pixel, const int *position) -> void
                            {
                                  float Z = curr_dep_img_[(position[0]*last_image->rows+ position[1])];
                                  this->egoVelocity(Z, position[0], position[1], &pixel); });
  // calculate the light change between the two frames
  last_image->forEach<float>([&](float &l1, const int *position) -> void
                             {
    float l2 = curr_image->at<float>(position[0], position[1]);
    this->lightChange(l1, l2, f_time_interval, &l1); });
  double temp_min_t_v, temp_min_t_b, max_;
  cv::Point min_loc, max_loc;
  // get the minimum value of the two
  cv::minMaxLoc(temp_image, &temp_min_t_v, &max_, &min_loc, &max_loc);
  cv::minMaxLoc(*last_image, &temp_min_t_b, &max_, &min_loc, &max_loc);

  *min_t_v = static_cast<float>(temp_min_t_v);
  *min_t_b = static_cast<float>(temp_min_t_b);
}

// void Esim::fillEvents(cv::Mat *mask, int polarity, std::vector<dvs_msgs::Event> *events)
void Esim::fillEvents(cv::Mat *mask, int polarity, std::vector<dvs_msgs::Event> *events)
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

void Esim::_debug_fillEvents(int x, int y, ros::Time ts, int p, std::vector<dvs_msgs::Event> *events)
{
  dvs_msgs::Event event;
  event.x = x;
  event.y = y;
  event.ts = ts;
  event.polarity = p;
  events->push_back(event);
}