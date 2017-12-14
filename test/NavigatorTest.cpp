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
 *  @file NavigatorTest.cpp
 *
 *  @brief Contains tests for navigator class
 *
 *  @author Pranav Inani
 *  @copyright 2017
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "terrapinavigator/Navigator.h"

/**
 * @brief Unit test to check initial linear motion
 *
 * Checks if the turtlebot has no movement in x direction
 * as we only want it to rotate initially
 */
TEST(NavigatorTest, InitialLinearTest) {
  Navigator nav = Navigator();
  EXPECT_EQ(0, nav.dir().linear.x);
}

/**
 * @brief Unit test to check initial angular rotation
 *
 * Checks if the action initially is to rotate the
 * turtlebot in place to explore environment
 */
TEST(NavigatorTest, InitialAngularTest) {
  Navigator nav = Navigator();
  EXPECT_NEAR(6.28, nav.dir().angular.z, 0.1);
}

/**
 * @brief Unit test for scan callback
 *
 * Checks if the obstacle distance is in the desired range
 */
TEST(NavigatorTest, CallbackTest) {
  ros::NodeHandle tn;
  Navigator nav = Navigator();
  ros::Subscriber subLaserScan = tn.subscribe
  ("/scan", 1000, &Navigator::ScanCallback, &nav);
  EXPECT_LE(nav.getObstDist(), 10);
}

/**
 * @brief Unit test for rotate flag
 *
 * Checks the setter and getter for rotate flag
 */
TEST(NavigatorTest, RotateFlagTest) {
  Navigator nav = Navigator();
  nav.setRotateFlag();
  EXPECT_EQ(nav.getRotateFlag(), false);
}
/**
 * @brief Unit test to check timer callback functionality
 *
 * Checks if the timer callback was called and the necessary
 * flag was set
 */
TEST(NavigatorTest, InitialRotateTest) {
  ros::NodeHandle tn;
  Navigator nav = Navigator();
  EXPECT_EQ(nav.getRotateFlag(), true);
}
/**
 * @brief Unit test for random angle rotation
 *
 * Checks if the robot moves with the desired
 * range of random angle when obstacle is detected
 */
TEST(NavigatorTest, RandomAngleTest) {
  Navigator nav = Navigator();
  nav.setRotateFlag();
  EXPECT_GE(101* (3.14 / 180), nav.dir().angular.z);
}

