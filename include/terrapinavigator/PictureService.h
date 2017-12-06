#include <string>
#include <vector>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <terrapinavigator/pictureService.h>

/**
 * @brief Camera class handles viewing onboard imagery and taking images
 */
class Camera {
 public:
  /**
   * @brief Camera constructor
   */
  Camera();

  /**
   * @brief Set the image flag so that the next time a camera topic is seen, we take a picture
   */
  bool takeImage(terrapinavigator::pictureService::Request &req,
                 terrapinavigator::pictureService::Response &resp);

  /**
   * @brief Camera topic callback takes a picture if flag has been set
   */
  void cameraCallback(const sensor_msgs::ImageConstPtr& msg);

 private:
  /**
   * @brief container for the filenames of each saved image
   */
  std::vector<std::string> savedImages;

  /**
   * @brief Flag denoting whether or not to take an image on next receipt of camera topic
   */
  bool takeImageFlag;

  /**
   * @brief Container for takeImage service
   */
  ros::ServiceClient cameraClient;

  /**
   * @brief Node handler for subscribing to service and topics
   */
  ros::NodeHandle nh;
};
