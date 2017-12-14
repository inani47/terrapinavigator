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
 *  @file walker.h
 *
 *  @brief Header file of walker class
 *
 *  @author Pranav Inani
 *  @copyright 2017
 */
#ifndef INCLUDE_NAVIGATOR_H_
#define INCLUDE_NAVIGATOR_H_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

/**
 * @brief Navigator class
 *
 * Generates twist messages for turtlebot to explore
 * Starts by rotating in place then moving ahead
 * Rotates in place every 45 seconds to aid mapping
 * Avoids obstacles by rotating with random angles till clear
 *
 */

class Navigator {
 public:
  /**
   * @brief constuctor for Navigator Class
   */
  Navigator();
  /**
   *
   * @brief getter for obstDist
   *
   * @return obstDist
   *
   */
  float getObstDist();
  /**
   *
   * @brief getter for rotateFlag
   *
   * @return rotateFlag
   *
   */
  bool getRotateFlag();
  /**
   *
   * @brief setter for rotateFlag
   *
   * sets the rotateFlag to false
   *
   * @return none
   *
   */
  void setRotateFlag();
  /**
   * @brief callback function for the camera subscriber
   *
   * finds the closest obstacle
   *
   * @param input is the pointer to array containing obstacle distances
   *
   * @return none
   */
  void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& input);
  /**
   * @brief callback function for the rotate timer
   *
   * sets the take rotate flag to true every 45 seconds
   *
   * @param event is the ros::TimerEvent structure it provides
   *        timing information useful when debugging.
   *
   * @return none
   */
  void RotatetimerCallback(const ros::TimerEvent& event);
  /**
   * @brief callback function for the cam timer
   *
   * Generates twist messages for turtleBot to moce
   *
   * @return action (the twist messages)
   */
  geometry_msgs::Twist dir();
  /**
   * @brief Destructor for Navigator Class
   */
  ~Navigator();

 private:
  /**
   * @brief variable that contains the distance to obstacle
   */
  float obstDist;
  /**
   * @brief flag for rotating the turtleBot
   */
  bool rotateFlag;
  /**
   * @brief counts the number of rotations
   */
  int rotateCount;
  /**
   * @brief twist message for turtleBot to move
   */
  geometry_msgs::Twist action;
};
#endif // INCLUDE_NAVIGATOR_H_

