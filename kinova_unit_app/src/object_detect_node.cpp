#include <ros/ros.h>

#include <object_detect.h>

int main (int argc, char** argv) {
  ros::init (argc, argv, "object_detect_node");

  ros::NodeHandle nh, pnh("~");
  ObjectDetect object_detect(&nh, &pnh);

  ros::spin();
}
