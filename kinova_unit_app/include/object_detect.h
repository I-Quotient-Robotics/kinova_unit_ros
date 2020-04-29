#ifndef OBJECT_DETECT_NODE_H_
#define OBJECT_DETECT_NODE_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include "kinova_unit_app/GetObject.h"

class ObjectDetect {
  public:
    explicit ObjectDetect(ros::NodeHandle* nh, ros::NodeHandle* pnh);

  private:
    void PointCloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    bool ObjectRequestCB(kinova_unit_app::GetObject::Request &req, kinova_unit_app::GetObject::Response &res);

    ros::NodeHandle *node_handle_;
    ros::NodeHandle *private_node_handle_;

    geometry_msgs::PoseStamped latest_object_pose_;

    std::string point_cloud_frame_;

    std::string point_cloud_topic_;
    std::string debug_output_topic_;
    std::string object_visual_markers_topic_;

    ros::Subscriber point_cloud_sub_;
    ros::Publisher debug_pointcloud_pub_;
    ros::ServiceServer object_request_srv_;

    ros::Publisher object_pub_;

    rviz_visual_tools::RvizVisualToolsPtr object_visual_markers_pub_;
};

#endif // OBJECT_DETECT_NODE_H_
