#include <object_detect.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

ObjectDetect::ObjectDetect(ros::NodeHandle *nh, ros::NodeHandle *pnh) : node_handle_(nh), private_node_handle_(pnh) {
  private_node_handle_->param<std::string>("point_cloud_frame", point_cloud_frame_, "");
  private_node_handle_->param<std::string>("point_cloud_topic", point_cloud_topic_, "/depth/pointcloud");
  private_node_handle_->param<std::string>("debug_output_topic", debug_output_topic_, "/debug_pointcloud_output");
  private_node_handle_->param<std::string>("object_visual_markers_topic", object_visual_markers_topic_, "/object_visual_markers");

  point_cloud_sub_ = node_handle_->subscribe (point_cloud_topic_, 1, &ObjectDetect::PointCloudCB, this);
  debug_pointcloud_pub_ = node_handle_->advertise<sensor_msgs::PointCloud2> (debug_output_topic_, 1);

  object_pub_ = node_handle_->advertise<geometry_msgs::PoseArray> ("object_pose", 1);

  object_request_srv_ = private_node_handle_->advertiseService("request_first_object", &ObjectDetect::ObjectRequestCB, this);

  object_visual_markers_pub_.reset(new rviz_visual_tools::RvizVisualTools(point_cloud_frame_, object_visual_markers_topic_));
  object_visual_markers_pub_->loadMarkerPub();
  object_visual_markers_pub_->deleteAllMarkers();
  object_visual_markers_pub_->enableBatchPublishing();
}

bool ObjectDetect::ObjectRequestCB(kinova_unit_app::GetObject::Request &req, kinova_unit_app::GetObject::Response &res) {
  if(req.request_type == 0) {
    if(ros::Time::now() - latest_object_pose_.header.stamp < ros::Duration(0.5)) {
      res.object_pose = latest_object_pose_;
    } else {
      ROS_INFO("no update for object for %lf", (ros::Time::now()-latest_object_pose_.header.stamp).toSec());
      return false;
    }
  }

  return true;
}

void ObjectDetect::PointCloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  pcl::PCLPointCloud2 debugPCL;
  sensor_msgs::PointCloud2 debug_output;

  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform voxel grid downsampling filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  // sor.setLeafSize (0.005, 0.005, 0.005);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (*cloudFilteredPtr);

  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud); // need a boost shared pointer for pcl function inputs

  // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

  //perform passthrough filtering to remove table leg

  // create a pcl object to hold the passthrough filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass_x, pass_y, pass_z;
  pass_x.setInputCloud (xyzCloudPtr);
  pass_x.setFilterFieldName ("x");
  pass_x.setFilterLimits (-0.4, 0.2);
  pass_x.filter (*xyzCloudPtrFiltered);

  pass_y.setInputCloud (xyzCloudPtrFiltered);
  pass_y.setFilterFieldName ("y");
  pass_y.setFilterLimits (-0.3, 0.3);
  pass_y.filter (*xyzCloudPtrFiltered);

  pass_z.setInputCloud (xyzCloudPtrFiltered);
  pass_z.setFilterFieldName ("z");
  pass_z.setFilterLimits (0.85, 1.05);
  pass_z.filter (*xyzCloudPtrFiltered);
  // writer.write ("table_scene_mug_stereo_textured_plane.pcd", *xyzCloudPtrFiltered, false);

  // pcl::toPCLPointCloud2( *xyzCloudPtrFiltered ,debugPCL);
  // pcl_conversions::fromPCL(debugPCL, debug_output);
  // debug_pointcloud_pub_.publish(debug_output);

  // create a pcl object to hold the ransac filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);

  // perform ransac planar filtration to remove table top
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
  // Optional
  seg1.setOptimizeCoefficients (true);
  // Mandatory
  seg1.setModelType (pcl::SACMODEL_PLANE);
  seg1.setMethodType (pcl::SAC_RANSAC);
  seg1.setDistanceThreshold (0.023);

  seg1.setInputCloud (xyzCloudPtrFiltered);
  seg1.segment (*inliers, *coefficients);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  //extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*xyzCloudPtrRansacFiltered);

  pcl::toPCLPointCloud2( *xyzCloudPtrRansacFiltered ,debugPCL);
  pcl_conversions::fromPCL(debugPCL, debug_output);
  debug_pointcloud_pub_.publish(debug_output);

  // perform euclidean cluster segmentation to seporate individual objects

  if(xyzCloudPtrRansacFiltered->empty() == true) {
    return;
  }

  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (xyzCloudPtrRansacFiltered);

  // create the extraction object for the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // specify euclidean cluster parameters
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (xyzCloudPtrRansacFiltered);
  // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
  ec.extract (cluster_indices);

  // declare the output variable instances
  sensor_msgs::PointCloud2 output;
  pcl::PCLPointCloud2 outputPCL;

  // ROS_INFO_STREAM(cluster_indices.size());
  Eigen::Isometry3d top_pose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d centroid_pose = Eigen::Isometry3d::Identity();
  object_visual_markers_pub_->deleteAllMarkers();
  geometry_msgs::PoseArray  pose_array;
  // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {

    // create a pcl object to hold the extracted cluster
    pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);

    // now we are in a vector of indices pertaining to a single cluster.
    // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
      clusterPtr->points.push_back(xyzCloudPtrRansacFiltered->points[*pit]);
    }

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*clusterPtr, centroid);

    Eigen::Vector4f minPoint, maxPoint;
    pcl::getMinMax3D(*clusterPtr, minPoint, maxPoint);

    top_pose.translation() << (maxPoint[0]+minPoint[0])/2.0, (maxPoint[1]+minPoint[1])/2.0, minPoint[2];
    centroid_pose.translation() << (maxPoint[0]+minPoint[0])/2.0, (maxPoint[1]+minPoint[1])/2.0, (maxPoint[2]+minPoint[2])/2.0;
    object_visual_markers_pub_->publishAxis(top_pose);
    object_visual_markers_pub_->publishWireframeCuboid(centroid_pose, maxPoint[0]-minPoint[0], maxPoint[1]-minPoint[1], maxPoint[2]-minPoint[2], rviz_visual_tools::GREEN);

    geometry_msgs::Pose pose;
    pose.position.x = top_pose.translation()[0];
    pose.position.y = top_pose.translation()[1];
    pose.position.z = top_pose.translation()[2];

    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    pose_array.poses.push_back(pose);

    // convert to pcl::PCLPointCloud2
    pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);

    // Convert to ROS data type
    pcl_conversions::fromPCL(outputPCL, output);
  }

  pose_array.header.stamp = ros::Time::now();
  pose_array.header.frame_id = point_cloud_frame_;

  if(pose_array.poses.size() > 0) {
    latest_object_pose_.header.stamp = pose_array.header.stamp;
    latest_object_pose_.header.frame_id = pose_array.header.frame_id;
    latest_object_pose_.pose = pose_array.poses[0];
  }

  object_pub_.publish(pose_array);

  object_visual_markers_pub_->trigger();
}
