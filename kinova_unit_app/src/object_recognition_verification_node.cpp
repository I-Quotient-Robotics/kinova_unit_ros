#include "kinova_unit_app/object_recognition_verification.h"

int main (int argc, char** argv) {
  ros::init (argc, argv, "object_detect_node");

  ros::NodeHandle nh, pnh("~");
  ObjectDetect object_detect(&nh, &pnh);
  ros::spin();
}
