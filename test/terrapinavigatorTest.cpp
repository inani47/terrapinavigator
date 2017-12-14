/*
 * terrapinavigatorTest.cpp
 *
 *  Created on: Dec 13, 2017
 *      Author: pranav
 */
#include <ros/ros.h>
#include <gtest/gtest.h>



int main(int argc, char **argv) {
  ros::init(argc, argv, "terrapinavigatorTest");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();

}
