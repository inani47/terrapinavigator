#include <stdlib.h>
#include <ros/ros.h>
#include <terrapinavigator/PictureService.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sstream>
#include <string>

/**
 * @brief Camera constructor
 */
Camera::Camera()
    : takeImageFlag(false) {

  // Register client to "takeImage" service
  cameraClient = nh.serviceClient < terrapinavigator::pictureService
      > ("takeImage");
}

/**
 * @brief Camera topic callback takes a picture if flag has been set
 */
void Camera::cameraCallback(const sensor_msgs::ImageConstPtr& msg) {
  if (takeImageFlag) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // Save OpenCV Image to file:
    cv::namedWindow("Image window");
    cv::imshow("Image window", cv_ptr->image);
    std::ostringstream filename;
    filename << "turtleBotImage.jpg";
    cv::imwrite(filename.str(), cv_ptr->image);

    ROS_INFO("Saving image %s to ~/.ros/", filename.str().c_str());

    // Add filename to list of saved images:
    savedImages.push_back(filename.str());

    // Reset Flag:
    takeImageFlag = false;
  }
}

/**
 * @brief Set the image flag so that the next time a camera topic is seen, we take a picture
 */
bool Camera::takeImage(terrapinavigator::pictureService::Request &req,
                       terrapinavigator::pictureService::Response &resp) {
  resp.resp = true;

  ROS_INFO_STREAM("Save the next available image frame.");

  takeImageFlag = resp.resp;

  return true;
}


