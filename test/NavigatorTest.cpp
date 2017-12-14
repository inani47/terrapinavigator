/*
 * NavigatorTest.cpp
 *
 *  Created on: Dec 13, 2017
 *      Author: pranav
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
  ros::Subscriber subLaserScan = tn.subscribe("/scan", 1000, &Navigator::ScanCallback, &nav);
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


