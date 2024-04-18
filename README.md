# DVS Gazebo Plugin

This package provides a DVS simulation implemented as Gazebo plugin.

## Install

First, make sure the DVS datatypes are available in your installation.
For this, clone the [RPG DVS ROS](https://github.com/uzh-rpg/rpg_dvs_ros) package into your catkin workspace.

Then, clone this package into your workspace and rebuild.

## Usage

This plugin can be used as a drop-in replacement for normal Gazebo camera plugins.
Both, the DVS plugin and the [CameraPlugin](https://bitbucket.org/osrf/gazebo/src/666bf30ad9a3c042955b55f79cf1a5416a70d83d/plugins/CameraPlugin.cc)
use the Gazebo [CameraSensor](https://bitbucket.org/osrf/gazebo/src/666bf30ad9a3c042955b55f79cf1a5416a70d83d/gazebo/sensors/CameraSensor.cc) internally.

The following SDF snippet shows an example usage:
```xml
<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='event_camera'>
  <pose>0 0 0 0 0 0</pose>
   <link name="link">
        <inertial>
          <mass>0.015</mass>
          <inertia>
            <ixx>4.15e-6</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>2.407e-6</iyy>
          <iyz>0</iyz>
          <izz>2.407e-6</izz>
          </inertia>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.01 0.01 0.01</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.01 0.01 0.01</size>
            </box>
          </geometry>
        </visual>
    <sensor name='camera' type='camera'>
        <camera name='__default__'>
            <horizontal_fov>1.8</horizontal_fov>
            <image>
                <width>128</width>
                <height>128</height>
            </image>
            <clip>
                <near>0.1</near>
                <far>100</far>
            </clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>400</update_rate>
        <visualize>1</visualize>
        <plugin name='camera_controller' filename='libgazebo_dvs_plugin.so'>
            <sensorName>dvs</sensorName>
            <robotNamespace>/</robotNamespace>
            <eventThreshold>20</eventThreshold>
            <eventsTopicName>events</eventsTopicName>
            <imageTopicName>image_raw</imageTopicName>
            <!-- <imuTopicName>imu</imuTopicName> -->
            <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        </plugin>
    </sensor>
    <self_collide>0</self_collide>
   <kinematic>0</kinematic>
  </link>
  </model>
</sdf>
```

The parameters `robotNamespace`, `cameraName` and `eventsTopicName` (default: "events") result in `"$robotNamespace/$cameraName/$eventsTopicName"`
as the identifier of the provided events topic.
In this case, events will be accessible from `"/AADC_AudiTT/camera_front/events"`.

The parameter `eventThreshold` specifies the pixel-wise threshold which has to be exceeded for a event to be emitted for this pixel.

The sensor parameter `update_rate` has only limited effect in Gazebo.
The real rate is determined by the rendering pipeline and can be way lower than the specified rate.
Still, this implementation yields a higher event frequency than similar Python-based implementations as a standalone node.

# Acknowledgement

We thank paper [Towards a framework for end-to-end control of a simulated vehicle with spiking neural networks](http://ieeexplore.ieee.org/document/7862386/) for their open source, please cite there paper if you use this code.

```
@INPROCEEDINGS{7862386,
author={J. Kaiser and J. C. V. Tieck and C. Hubschneider and P. Wolf and M. Weber and M. Hoff and A. Friedrich and K. Wojtasik and A. Roennau and R. Kohlhaas and R. Dillmann and J. M. Zöllner},
booktitle={2016 IEEE International Conference on Simulation, Modeling, and Programming for Autonomous Robots (SIMPAR)},
title={Towards a framework for end-to-end control of a simulated vehicle with spiking neural networks},
year={2016},
pages={127-134},
keywords={automobiles;cameras;complex networks;feedforward neural nets;learning (artificial intelligence);mobile robots;DVS;camera images;complex networks;deep learning architectures;end-to-end simulated vehicle control;hand-crafted feature detectors;neural self-driving vehicle applications;neurorobotics applications;rate-based neural networks;silicon retina;spiking neural networks;steering wheel decoder;vehicle end-to-end for lane following behavior;Biological neural networks;Brain modeling;Cameras;Robot sensing systems;Voltage control},
doi={10.1109/SIMPAR.2016.7862386},
month={Dec},}
```
