/*
 *BSD 3-Clause License
 *
 *Copyright (c) 2017, Pranav Inani
 *All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 *  @file Navigator.cpp
 *
 *  @brief Implementation of Navigator class
 *
 *  @author Pranav Inani
 *  @copyright 2017
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "terrapinavigator/Navigator.h"

Navigator::Navigator()
    : rotateFlag(true),
      obstDist(0),
      rotateCount(0) {
}

float Navigator::getObstDist() {
  return obstDist;
}

bool Navigator::getRotateFlag() {
  return rotateFlag;
}

void Navigator::setRotateFlag() {
  rotateFlag = false;
}

void Navigator::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& input) {
  float min = 10;
  for (const auto& dist : input->ranges) {
    if (min > dist) {
      min = dist;
    }
  }
  obstDist = min;  // Finds distance to obstacle
  ROS_DEBUG_STREAM("Distance to obstacle: " << obstDist);
}

void Navigator::RotatetimerCallback(const ros::TimerEvent& event) {
  if (obstDist > .75) {
    // sets rotate flag every 45 seconds if no obstacle present
    rotateFlag = true;
    ROS_INFO_STREAM("Rotating in place to aid mapping");
  }
}

geometry_msgs::Twist Navigator::dir() {
  // Initialize the twist message
  action.linear.x = 0.0;
  action.linear.y = 0.0;
  action.linear.z = 0.0;
  action.angular.x = 0.0;
  action.angular.y = 0.0;
  action.angular.z = 0.0;
  rotateCount++;

  std::random_device rd;  // used to initialize (seed) engine //
  std::mt19937 rng(rd());  // random-number engine used (Mersenne-Twister) //
  std::uniform_int_distribution<int> uni(30, 100);  // guaranteed unbiased //
  float randomAngle = uni(rng);  // random number between 30 and 100
  // Stops and rotates the turtleBot in place 5 times
  if (rotateFlag) {
    action.angular.z = 360 * (3.14 / 180);
  }
  if (rotateCount == 5) {
    setRotateFlag();
    rotateCount = 0;
  }
  if (obstDist > 0.75 && rotateFlag == false) {
    //  Linear motion in forward direction
    action.linear.x = 0.25;
    ROS_INFO_STREAM_THROTTLE(5, "Moving Forward");
  } else if (obstDist < 0.75 && rotateFlag == false) {
    //  2D Rotation of about 90 degress
    action.angular.z = randomAngle * (3.14 / 180);
    ROS_WARN_STREAM(
        "OBSTACLE DETECTED! Turning randomly: "<< action.angular.z*(180/3.14));
  }
  return action;
}

Navigator::~Navigator() {
}
