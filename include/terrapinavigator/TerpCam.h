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
 *  @file TerpCam.h
 *
 *  @brief Header file for TerpCam class
 *
 *  @author Pranav Inani
 *  @copyright 2017
 */
#ifndef INCLUDE_TERPCAM_H_
#define INCLUDE_TERPCAM_H_

#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <terrapinavigator/pictureService.h>
#include <sensor_msgs/Image.h>


/**
 * @brief TerpCam class
 *
 * Implements picture service
 * Takes a picture every 40 seconds
 * Also can take pictures when desired by the user
 *
 */
class TerpCam {
 public:
  /**
   * @brief Destructor for TerpCam Class
   */
  TerpCam();
  /**
   *
   * @brief getter for takeImageFlag
   *
   * @return takeImageFlag
   *
   */
  bool getTakeImageFlag();
  /**
   *
   * @brief setter for takeImageFlag
   *
   * sets the takeImageFlag to true
   *
   * @return none
   *
   */
  void setTakeImageFlag();
  /**
   * @brief callback function for the take image server
   *
   * @param req is the request sent by the client
   * @param res is the response sent by the server
   *
   * @return true if the response is successful
   */
  bool takeImage(terrapinavigator::pictureService::Request &req,
                 terrapinavigator::pictureService::Response &resp);
  /**
   * @brief callback function for the cam timer
   *
   * sets the take image flag to true every 40 seconds
   *
   * @param event is the ros::TimerEvent structure it provides
   *        timing information useful when debugging.
   *
   * @return none
   */
  void camTimerCallback(const ros::TimerEvent& event);
  /**
   * @brief callback function for the camera subscriber
   *
   * saves an image locally if take image flag is true
   *
   * @param img is a constant image pointer of what the turtlebot is seeing
   *
   * @return none
   */
  void cameraCallback(const sensor_msgs::ImageConstPtr& img);
  /**
   * @brief Destructor for TerpCam Class
   */
  ~TerpCam();

 private:
  /**
   * @brief flag for taking image
   */
  bool takeImageFlag;
  /**
   * @brief registers client for camera service
   */
  ros::ServiceClient cameraClient;

  /**
   * @brief Node handle for camera service
   */
  ros::NodeHandle nh;
};

#endif // INCLUDE_TERPCAM_H_
