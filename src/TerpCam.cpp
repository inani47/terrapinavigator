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
 *  @file TerpCam.cpp
 *
 *  @brief Implementation of TerpCam class
 *
 *  @author Pranav Inani
 *  @copyright 2017
 */
#include <stdlib.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sstream>
#include <string>
#include "terrapinavigator/pictureService.h"
#include "terrapinavigator/TerpCam.h"

TerpCam::TerpCam()
    : takeImageFlag(false) {

  cameraClient = nh.serviceClient<terrapinavigator::pictureService>(
      "takeImage");
}

bool TerpCam::getTakeImageFlag() {
  return takeImageFlag;
}

void TerpCam::setTakeImageFlag() {
  takeImageFlag = true;
}

bool TerpCam::takeImage(terrapinavigator::pictureService::Request &req,
                        terrapinavigator::pictureService::Response &resp) {
  resp.clickImg = true;

  ROS_INFO_STREAM("Taking a picture");

  takeImageFlag = resp.clickImg;

  return true;
}

void TerpCam::camTimerCallback(const ros::TimerEvent& event) {
  takeImageFlag = true;  // sets take image flag every 40 seconds
  ROS_INFO_STREAM("Taking a picture");
}

void TerpCam::cameraCallback(const sensor_msgs::ImageConstPtr& img) {
  if (takeImageFlag) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR_STREAM("cv_bridge exception:" << e.what());
      return;
    }
    std::ostringstream filename;
    filename << "terpImage" << ros::WallTime::now()  // appends the current time to each file
        << ".jpg";
    cv::imwrite(filename.str(), cv_ptr->image);  // saves the file
    ROS_INFO_STREAM("Saving image " << filename.str().c_str() << "to ~/.ros/");
    takeImageFlag = false;
  }
}

TerpCam::~TerpCam() {
}

