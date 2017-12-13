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
 *  @file Turtle.cpp
 *
 *  @brief Implementation of Turtle class
 *
 *  @author Pranav Inani
 *  @copyright 2017
 */
#include "../include/terrapinavigator/Turtle.h"

#include <iostream>
#include <ros/ros.h>
#include <terrapinavigator/pictureService.h>
#include "terrapinavigator/Navigator.h"

Turtle::Turtle() {
  cameraSub = n.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 500,
                                              &TerpCam::cameraCallback, &cam);

  takeImageServer = n.advertiseService("pictureService", &TerpCam::takeImage,
                                       &cam);
  subLaserScan = n.subscribe("/scan", 1000, &Navigator::callback, &navigator);

  timer = n.createTimer(ros::Duration(40), &Navigator::timerCallback,
                        &navigator);
  camTimer = n.createTimer(ros::Duration(45), &TerpCam::camTimerCallback, &cam);
  actionPub = n.advertise<geometry_msgs::Twist
      > ("/mobile_base/commands/velocity", 100);
}

void Turtle::explore() {
  actionPub.publish(navigator.dir());
}
