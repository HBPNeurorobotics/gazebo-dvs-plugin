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
  event_threshold = 10.0;
  mem_last_image = cv::Mat::ones(0, 0, CV_32F);
}

Esim::Esim(float event_threshold, int width, int height)
{
  velocity.x = 0;
  velocity.y = 0;
  velocity.z = 0;
  angular_velocity.x = 0;
  angular_velocity.y = 0;
  angular_velocity.z = 0;
  this->event_threshold = event_threshold;
  last_time = ros::Time::now();
  mem_last_image = cv::Mat::ones(width, height, CV_32F);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Esim::~Esim()
{
}

// calibrate the static bias of the imu
void Esim::imuCalibration(const std::vector<sensor_msgs::Imu> *imu_msgs)
{
  int imu_msg_size = imu_msgs->size();
  int amount = imu_msg_size;
  // 使用最小二乘的回归计算偏置和倍缩，但是倍缩必须在标准速度下校准，这里做不出来，就用时漂去平替它
  float sum_w_x = 0, sum_w_y = 0, sum_w_z = 0;
  float sum_a_x = 0, sum_a_y = 0, sum_a_z = 0;
  for (int i = 0; i < imu_msg_size; i++)
  {
    if (imu_msgs->at(i).linear_acceleration_covariance[0] < 0 || imu_msgs->at(i).angular_velocity_covariance[0] < 0)
    {
      amount--;
      continue;
    }
    sum_w_x += imu_msgs->at(i).angular_velocity.x;
    sum_w_y += imu_msgs->at(i).angular_velocity.y;
    sum_w_z += imu_msgs->at(i).angular_velocity.z;

    sum_a_x += imu_msgs->at(i).linear_acceleration.x;
    sum_a_y += imu_msgs->at(i).linear_acceleration.y;
    sum_a_z += imu_msgs->at(i).linear_acceleration.z;
  }

  this->bias_w_x = sum_w_x / amount;
  this->bias_w_y = sum_w_y / amount;
  this->bias_w_z = sum_w_z / amount;

  this->bias_a_x = sum_a_x / amount;
  this->bias_a_y = sum_a_y / amount;
  this->bias_a_z = sum_a_z / amount;
}
void Esim::imuReoutput(const sensor_msgs::Imu &imu_msg, sensor_msgs::Imu &imu_msg_out)
{

  imu_msg_out.linear_acceleration.x = imu_msg.linear_acceleration.x - this->bias_a_x;
  imu_msg_out.linear_acceleration.y = imu_msg.linear_acceleration.y - this->bias_a_y;
  imu_msg_out.linear_acceleration.z = imu_msg.linear_acceleration.z - this->bias_a_z;
  imu_msg_out.angular_velocity.x = imu_msg.angular_velocity.x - this->bias_w_x;
  imu_msg_out.angular_velocity.y = imu_msg.angular_velocity.y - this->bias_w_y;
  imu_msg_out.angular_velocity.z = imu_msg.angular_velocity.z - this->bias_w_z;
}
void Esim::setEventThreshold(const float event_threshold)
{
  this->event_threshold = event_threshold;
}

// last_image : the last time after event was created 32UC1
// curr_image : the current image 32UC1
void Esim::simulateESIM(cv::Mat *last_iamge, const cv::Mat *curr_image, std::vector<dvs_msgs::Event> *events, const sensor_msgs::Imu &imu_msg, sensor_msgs::Image &msg_dep_img, const ros::Time &current_time, const ros::Time &last_time)
{
  cv::Mat last_image_ = this->mem_last_image.clone();
  cv::Mat curr_image_ = curr_image->clone();

  float f_time_interval = current_time.toSec() - last_time.toSec();

  curr_image_.convertTo(curr_image_, CV_32F);

  curr_image_ += 1e-4;
  last_image_ += 1e-4;

  // get the imu and depth messages

  assert(msg_dep_img.encoding == sensor_msgs::image_encodings::TYPE_32FC1);

  // convert the depth image from senso::Image to float
  assert(msg_dep_img.data.size() % sizeof(float) == 0);
  const float *depth_image = reinterpret_cast<float *>(msg_dep_img.data.data());

  if (imu_msg.orientation_covariance[0] < 0)
  {
    return;
  }
  // calculate the timestamp between two consecutive frames
  sensor_msgs::Imu imu_msg_out;
  this->imuReoutput(imu_msg, imu_msg_out);

  this->velocity.x += (imu_msg_out.linear_acceleration.x) * f_time_interval;
  this->velocity.y += (imu_msg_out.linear_acceleration.y) * f_time_interval;
  this->velocity.z += (imu_msg_out.linear_acceleration.z) * f_time_interval;
  this->angular_velocity.x = (imu_msg_out.angular_velocity.x);
  this->angular_velocity.y = (imu_msg_out.angular_velocity.y);
  this->angular_velocity.z = (imu_msg_out.angular_velocity.z);

  float max_t_v, max_t_l;
  this->adaptiveSample(&last_image_, &curr_image_, depth_image, f_time_interval, &max_t_v, &max_t_l);
  float lambda_b = 0.5, lambda_v = 0.5;
  // the sample interval between two events for events generation is 1 us.
  float t_sample_interval = std::min(lambda_b * max_t_l, lambda_v * max_t_v);
  // caculate the slope matrix between the two frames
  cv::Mat slope = (curr_image_ - this->mem_last_image) / f_time_interval;

  // convert to log intensity to store the last time after event was created
  if (t_sample_interval >= f_time_interval)
  {
    this->processDelta(&this->mem_last_image, &curr_image_, events);
  }
  else
  {
    for (int iter_num = 0; iter_num < static_cast<int>(f_time_interval / t_sample_interval); iter_num++)
    {
      // convert to log intensity to store the current image during the interpolation
      last_image_ += slope * t_sample_interval;
      this->processDelta(&this->mem_last_image, &last_image_, events);
    }
  }
}

// last_image : the last time after event was created
// curr_image : the current image
// this function will change the last_image in accordance with events activation after the event was created
void Esim::processDelta(cv::Mat *last_image, const cv::Mat *curr_image, std::vector<dvs_msgs::Event> *events)
{
  if (curr_image->size() == last_image->size())
  {
    cv::Mat pos_diff = *curr_image - *last_image;
    cv::Mat neg_diff = *last_image - *curr_image;

    cv::Mat pos_mask;
    cv::Mat neg_mask;

    cv::threshold(pos_diff, pos_mask, this->event_threshold, 255, cv::THRESH_BINARY);
    cv::threshold(neg_diff, neg_mask, this->event_threshold, 255, cv::THRESH_BINARY);

    last_image->forEach<float>([&](float &pixel, const int *position) -> void
                               {
                                 if (pos_mask.at<float>(position[0], position[1]) >0)
                                 {
                                   pixel = curr_image->at<float>(position[0], position[1]);
                                 }
                                 if (neg_mask.at<float>(position[0], position[1]) >0)
                                 {
                                   pixel = curr_image->at<float>(position[0], position[1]);
                                 } });

    pos_mask.convertTo(pos_mask, CV_32S);
    neg_mask.convertTo(neg_mask, CV_32S);

    this->fillEvents(&pos_mask, 0, events);
    this->fillEvents(&neg_mask, 1, events);
  }
  else
  {
    gzwarn << "Unexpected change in image size (" << last_image->size() << " -> " << curr_image->size() << "). Publishing no events for this frame change." << std::endl;
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
  *delta_pixel = std::fabs((last_pixel - curr_pixel) / f_time_interval);
}

void Esim::adaptiveSample(const cv::Mat *last_image, const cv::Mat *curr_image, const float *curr_dep_img_, const float f_time_interval, float *max_t_v, float *max_t_l)
{
  cv::Mat last_image_(last_image->size(), CV_32F), temp_image_(last_image->size(), CV_32F);
  // calculate the velocity and angular velocity for the camera ego movement
  temp_image_.forEach<float>([&](float &pixel, const int *position) -> void
                             {
                                  float Z = curr_dep_img_[(position[0]*last_image->rows+ position[1])];
                                  this->egoVelocity(Z, position[0], position[1], &pixel); });
  // calculate the light change between the two frames
  last_image_.forEach<float>([&](float &p, const int *position) -> void
                             {
       float l1 = last_image->at<float>(position[0], position[1]);                       
    float l2 = curr_image->at<float>(position[0], position[1]);
    this->lightChange(l1, l2, f_time_interval, &p); });
  double temp_max_t_v, temp_max_t_l, min_;
  cv::Point min_loc, max_loc;
  // get the minimum value of the two
  cv::minMaxLoc(temp_image_, &min_, &temp_max_t_v, &min_loc, &max_loc);
  cv::minMaxLoc(last_image_, &min_, &temp_max_t_l, &min_loc, &max_loc);

  // max_t_b is the change of the light intensity between the two frames
  *max_t_v = static_cast<float>(1 / temp_max_t_v);
  *max_t_l = static_cast<float>(1 / temp_max_t_l);
}

// void Esim::fillEvents(cv::Mat *mask, int polarity, std::vector<dvs_msgs::Event> *events)
void Esim::fillEvents(const cv::Mat *mask, const int polarity, std::vector<dvs_msgs::Event> *events)
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
