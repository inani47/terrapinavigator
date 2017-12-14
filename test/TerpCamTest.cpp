#include <ros/ros.h>
#include <gtest/gtest.h>
#include "terrapinavigator/TerpCam.h"




TEST(TerpCamTest, setImageFlagTest) {
  TerpCam c = TerpCam();
  c.setTakeImageFlag();
  EXPECT_EQ(true, c.getTakeImageFlag());
}

TEST(TerpCamTest, InitialImageFlagTest) {
  TerpCam c = TerpCam();
  EXPECT_EQ(false, c.getTakeImageFlag());
}



