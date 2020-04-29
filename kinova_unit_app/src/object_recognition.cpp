#ifndef OBJECT_DETECT_NODE_H_
#define OBJECT_DETECT_NODE_H_

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

#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include "kinova_unit_app/GetObject.h"

typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

std::string model_filename_;
std::string scene_filename_;

//Algorithm params
bool show_keypoints_ (false);
bool show_correspondences_ (false);
bool use_cloud_resolution_ (false);
bool use_hough_ (true);
float model_ss_ (0.01f);
float scene_ss_ (0.03f);
float rf_rad_ (0.015f);
float descr_rad_ (0.02f);
float cg_size_ (0.01f);
float cg_thresh_ (5.0f);

class ObjectDetect {
  public:
    explicit ObjectDetect(ros::NodeHandle* nh, ros::NodeHandle* pnh);
    void ObjectRecognition(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &xyzCloudPtrRansacFiltered);
    int writeornot = 0;
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


void
showHelp (char *filename)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*             Correspondence Grouping Tutorial - Usage Guide              *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                     Show this help." << std::endl;
  std::cout << "     -k:                     Show used keypoints." << std::endl;
  std::cout << "     -c:                     Show used correspondences." << std::endl;
  std::cout << "     -r:                     Compute the model cloud resolution and multiply" << std::endl;
  std::cout << "                             each radius given by that value." << std::endl;
  std::cout << "     --algorithm (Hough|GC): Clustering algorithm used (default Hough)." << std::endl;
  std::cout << "     --model_ss val:         Model uniform sampling radius (default 0.01)" << std::endl;
  std::cout << "     --scene_ss val:         Scene uniform sampling radius (default 0.03)" << std::endl;
  std::cout << "     --rf_rad val:           Reference frame radius (default 0.015)" << std::endl;
  std::cout << "     --descr_rad val:        Descriptor radius (default 0.02)" << std::endl;
  std::cout << "     --cg_size val:          Cluster size (default 0.01)" << std::endl;
  std::cout << "     --cg_thresh val:        Clustering threshold (default 5)" << std::endl << std::endl;
}

void
parseCommandLine (int argc, char *argv[])
{
  //Show help
  if (pcl::console::find_switch (argc, argv, "-h"))
  {
    showHelp (argv[0]);
    exit (0);
  }

  //Model & scene filenames
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (filenames.size () != 2)
  {
    std::cout << "Filenames missing.\n";
    showHelp (argv[0]);
    exit (-1);
  }

  model_filename_ = argv[filenames[0]];
  scene_filename_ = argv[filenames[1]];

  //Program behavior
  if (pcl::console::find_switch (argc, argv, "-k"))
  {
    show_keypoints_ = true;
  }
  if (pcl::console::find_switch (argc, argv, "-c"))
  {
    show_correspondences_ = true;
  }
  if (pcl::console::find_switch (argc, argv, "-r"))
  {
    use_cloud_resolution_ = true;
  }

  std::string used_algorithm;
  if (pcl::console::parse_argument (argc, argv, "--algorithm", used_algorithm) != -1)
  {
    if (used_algorithm.compare ("Hough") == 0)
    {
      use_hough_ = true;
    }else if (used_algorithm.compare ("GC") == 0)
    {
      use_hough_ = false;
    }
    else
    {
      std::cout << "Wrong algorithm name.\n";
      showHelp (argv[0]);
      exit (-1);
    }
  }

  //General parameters
  pcl::console::parse_argument (argc, argv, "--model_ss", model_ss_);
  pcl::console::parse_argument (argc, argv, "--scene_ss", scene_ss_);
  pcl::console::parse_argument (argc, argv, "--rf_rad", rf_rad_);
  pcl::console::parse_argument (argc, argv, "--descr_rad", descr_rad_);
  pcl::console::parse_argument (argc, argv, "--cg_size", cg_size_);
  pcl::console::parse_argument (argc, argv, "--cg_thresh", cg_thresh_);
}

double
computeCloudResolution (const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    if (! std::isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

void ObjectDetect::ObjectRecognition(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &xyzCloudPtrFiltered) {

  // if (use_cloud_resolution_)
  // {
  //   float resolution = static_cast<float> (computeCloudResolution (model));
  //   if (resolution != 0.0f)
  //   {
  //     model_ss_   *= resolution;
  //     scene_ss_   *= resolution;
  //     rf_rad_     *= resolution;
  //     descr_rad_  *= resolution;
  //     cg_size_    *= resolution;
  //   }

  //   std::cout << "Model resolution:       " << resolution << std::endl;
  //   std::cout << "Model sampling size:    " << model_ss_ << std::endl;
  //   std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
  //   std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
  //   std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
  //   std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
  // }

  // //
  // //  Compute Normals
  // //
  // pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  // norm_est.setKSearch (10);
  // norm_est.setInputCloud (model);
  // norm_est.compute (*model_normals);

  // norm_est.setInputCloud (scene);
  // norm_est.compute (*scene_normals);





}

void ObjectDetect::PointCloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  pcl::PCLPointCloud2 debugPCL;
  sensor_msgs::PointCloud2 debug_output;

  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2Ptr cloudPtr(cloud);
  pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);

  pcl::PCDWriter writer;

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
  pcl::fromPCLPointCloud2(*cloudPtr, *xyzCloudPtr);

  //perform passthrough filtering to remove table leg

  // create a pcl object to hold the passthrough filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass_x, pass_y, pass_z;
  pass_x.setInputCloud (xyzCloudPtr);
  pass_x.setFilterFieldName ("x");
  pass_x.setFilterLimits (-0.35, 0.15);
  pass_x.filter (*xyzCloudPtrFiltered);

  pass_y.setInputCloud (xyzCloudPtrFiltered);
  pass_y.setFilterFieldName ("y");
  pass_y.setFilterLimits (-0.3, 0.3);
  pass_y.filter (*xyzCloudPtrFiltered);

  pass_z.setInputCloud (xyzCloudPtrFiltered);
  pass_z.setFilterFieldName ("z");
  pass_z.setFilterLimits (0.85, 1.08);
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
  if(writeornot == 0) {

    writer.write ("/home/iqr/object3.pcd", *xyzCloudPtrRansacFiltered, false);
    ROS_INFO("write");
  }
  writeornot += 1;
  // ObjectRecognition(xyzCloudPtrRansacFiltered);





















  // // Create the KdTree object for the search method of the extraction
  // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  // tree->setInputCloud (xyzCloudPtrRansacFiltered);

  // // create the extraction object for the clusters
  // std::vector<pcl::PointIndices> cluster_indices;
  // pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // // specify euclidean cluster parameters
  // ec.setClusterTolerance (0.02); // 2cm
  // ec.setMinClusterSize (50);
  // ec.setMaxClusterSize (25000);
  // ec.setSearchMethod (tree);
  // ec.setInputCloud (xyzCloudPtrRansacFiltered);
  // // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
  // ec.extract (cluster_indices);

  // // declare the output variable instances
  // sensor_msgs::PointCloud2 output;
  // pcl::PCLPointCloud2 outputPCL;

  // // ROS_INFO_STREAM(cluster_indices.size());
  // Eigen::Isometry3d top_pose = Eigen::Isometry3d::Identity();
  // Eigen::Isometry3d centroid_pose = Eigen::Isometry3d::Identity();
  // object_visual_markers_pub_->deleteAllMarkers();
  // geometry_msgs::PoseArray  pose_array;
  // // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately
  // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {

  //   // create a pcl object to hold the extracted cluster
  //   pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
  //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);

  //   // now we are in a vector of indices pertaining to a single cluster.
  //   // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
  //   for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {
  //     clusterPtr->points.push_back(xyzCloudPtrRansacFiltered->points[*pit]);
  //   }

  //   Eigen::Vector4f centroid;
  //   pcl::compute3DCentroid(*clusterPtr, centroid);

  //   Eigen::Vector4f minPoint, maxPoint;
  //   pcl::getMinMax3D(*clusterPtr, minPoint, maxPoint);

  //   top_pose.translation() << (maxPoint[0]+minPoint[0])/2.0, (maxPoint[1]+minPoint[1])/2.0, minPoint[2];
  //   centroid_pose.translation() << (maxPoint[0]+minPoint[0])/2.0, (maxPoint[1]+minPoint[1])/2.0, (maxPoint[2]+minPoint[2])/2.0;
  //   object_visual_markers_pub_->publishAxis(top_pose);
  //   object_visual_markers_pub_->publishWireframeCuboid(centroid_pose, maxPoint[0]-minPoint[0], maxPoint[1]-minPoint[1], maxPoint[2]-minPoint[2], rviz_visual_tools::GREEN);

  //   geometry_msgs::Pose pose;
  //   pose.position.x = top_pose.translation()[0];
  //   pose.position.y = top_pose.translation()[1];
  //   pose.position.z = top_pose.translation()[2];

  //   pose.orientation.x = 0;
  //   pose.orientation.y = 0;
  //   pose.orientation.z = 0;
  //   pose.orientation.w = 1;

  //   pose_array.poses.push_back(pose);

  //   // convert to pcl::PCLPointCloud2
  //   pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);

  //   // Convert to ROS data type
  //   pcl_conversions::fromPCL(outputPCL, output);
  // }

  // pose_array.header.stamp = ros::Time::now();
  // pose_array.header.frame_id = point_cloud_frame_;

  // if(pose_array.poses.size() > 0) {
  //   latest_object_pose_.header.stamp = pose_array.header.stamp;
  //   latest_object_pose_.header.frame_id = pose_array.header.frame_id;
  //   latest_object_pose_.pose = pose_array.poses[0];
  // }

  // object_pub_.publish(pose_array);

  // object_visual_markers_pub_->trigger();
}


int main (int argc, char** argv) {
  ros::init (argc, argv, "object_detect_node");

  ros::NodeHandle nh, pnh("~");
  ObjectDetect object_detect(&nh, &pnh);
  // parseCommandLine (argc, argv);
  ros::spin();
}
