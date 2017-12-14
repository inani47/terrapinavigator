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
 *  @file walker.cpp
 *
 *  @brief Implementation of Navigator class
 *
 *  @author Pranav Inani
 *  @copyright 2017
 */
/**
 *  @file main.cpp
 *
 *  @brief Demo for the project
 *
 *  @author Pranav Inani
 *  @copyright 2017
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "terrapinavigator/Navigator.h"
#include <random>

#include "../include/terrapinavigator/Turtle.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "terrapinavigator");
  ros::NodeHandle n;
  Turtle terrapin = Turtle();
  // sleep for 15 seconds while gazebo starts
  ros::Duration(15).sleep();

  ros::Rate loop_rate(2);
  while (ros::ok()) {

  terrapin.explore();
//  ros::Subscriber subLaserScan;
//  ros::Publisher pub;
  // geometry_msgs::Twist msg;
//  subLaserScan = n.subscribe("/scan", 1000, &Walker::callback, &walk);
//  pub = n.advertise < geometry_msgs::Twist
//      > ("/mobile_base/commands/velocity", 100);
//  ros::Rate loop_rate(2);
  // Initialize the twist messsage
//  int count = 0;
//  ros::Timer timer = n.createTimer(ros::Duration(45), &Walker::timerCallback,
//                                   &walk);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

