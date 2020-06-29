#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <aruco_msgs/MarkerArray.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

bool receive_data = false;
bool object_updated = false;

geometry_msgs::TransformStamped marker_transform, object_transform;

void ObjectMarkerPoseCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
  for(uint16_t i=0; i<msg->poses.size(); i++) {
    marker_transform.header.stamp = ros::Time::now();
    marker_transform.header.frame_id = msg->header.frame_id;
    marker_transform.child_frame_id = "marker";

    marker_transform.transform.translation.x = msg->poses[i].position.x;
    marker_transform.transform.translation.y = msg->poses[i].position.y;
    marker_transform.transform.translation.z = msg->poses[i].position.z;
    marker_transform.transform.rotation.x = msg->poses[i].orientation.x;
    marker_transform.transform.rotation.y = msg->poses[i].orientation.y;
    marker_transform.transform.rotation.z = msg->poses[i].orientation.z;
    marker_transform.transform.rotation.w = msg->poses[i].orientation.w;

    object_transform.header.stamp = ros::Time::now();
    object_transform.header.frame_id = "marker";
    object_transform.child_frame_id = "object";

    object_transform.transform.translation.x = 0.00;
    object_transform.transform.translation.y = 0.00;
    object_transform.transform.translation.z = -0.036;

    tf2::Quaternion q;
    q.setRPY(-M_PI/2.0, 0.0, -M_PI/2.0);
    object_transform.transform.rotation.x = q.x();
    object_transform.transform.rotation.y = q.y();
    object_transform.transform.rotation.z = q.z();
    object_transform.transform.rotation.w = q.w();

    object_updated = true;
  }
  receive_data = true;

  // ROS_INFO("update Marker pose");
}

void publish_object(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface) {
  std::vector<moveit_msgs::CollisionObject> objects(2);

  // object
  objects[0].id = "object";
  objects[0].header.frame_id = "object";

  objects[0].primitives.resize(1);
  objects[0].primitives[0].type = objects[0].primitives[0].BOX;
  objects[0].primitives[0].dimensions.resize(3);
  objects[0].primitives[0].dimensions[0] = 0.07;
  objects[0].primitives[0].dimensions[1] = 0.07;
  objects[0].primitives[0].dimensions[2] = 0.1;

  objects[0].primitive_poses.resize(1);
  objects[0].primitive_poses[0].position.x = 0.00;
  objects[0].primitive_poses[0].position.y = 0.00;
  objects[0].primitive_poses[0].position.z = 0.00;
  objects[0].primitive_poses[0].orientation.w = 1.0;

  objects[0].operation = objects[0].ADD;

  // table
  // objects[1].id = "table";
  // objects[1].header.frame_id = "object";

  // objects[1].primitives.resize(1);
  // objects[1].primitives[0].type = objects[1].primitives[0].BOX;
  // objects[1].primitives[0].dimensions.resize(3);
  // objects[1].primitives[0].dimensions[0] = 1.0;
  // objects[1].primitives[0].dimensions[1] = 3.0;
  // objects[1].primitives[0].dimensions[2] = 1.5;

  // objects[1].primitive_poses.resize(1);
  // objects[1].primitive_poses[0].position.z = 0.7;
  // objects[1].primitive_poses[0].position.y = 0.00;
  // objects[1].primitive_poses[0].position.x = 0.55;
  // objects[1].primitive_poses[0].orientation.w = 1.0;

  // objects[1].operation = objects[1].ADD;

  planning_scene_interface.applyCollisionObjects(objects);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "markder_detect_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::WallDuration(1.0).sleep();

  ros::Subscriber object_markers_sub = nh.subscribe("object_pose", 1000, ObjectMarkerPoseCallback);

  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ROS_INFO("Wait for first marker...");
  while(receive_data == false) {
    ros::WallDuration(1.0).sleep();
  }

  ROS_INFO("Start object tf publish...");
  tf2_ros::TransformBroadcaster br;
  while(ros::ok()) {
    if(object_updated == true) {
      object_updated = false;
      br.sendTransform(marker_transform);
      br.sendTransform(object_transform);
    }

    // publish_object(planning_scene_interface);
  }

  ROS_INFO("Stop object tf publish");
  ros::waitForShutdown();
  return 0;
}
