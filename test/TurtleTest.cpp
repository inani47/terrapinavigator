/*
 * TurtleTest.h
 *
 *  Created on: Dec 13, 2017
 *      Author: pranav
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "../include/terrapinavigator/Turtle.h"


TEST(TurtleTest, PublishTest) {
  Turtle t = Turtle();
  EXPECT_EQ(true, t.explore());
}
