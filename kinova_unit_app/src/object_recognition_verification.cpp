#ifndef OBJECT_DETECT_NODE_H_
#define OBJECT_DETECT_NODE_H_

#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <Eigen/Dense>

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


#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/hv/hv_go.h>
#include <pcl/registration/icp.h>
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

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

struct CloudStyle
{
    double r;
    double g;
    double b;
    double size;

    CloudStyle (double r,
                double g,
                double b,
                double size) :
        r (r),
        g (g),
        b (b),
        size (size)
    {
    }
};

CloudStyle style_white (255.0, 255.0, 255.0, 4.0);
CloudStyle style_red (255.0, 0.0, 0.0, 3.0);
CloudStyle style_green (0.0, 255.0, 0.0, 5.0);
CloudStyle style_cyan (93.0, 200.0, 217.0, 4.0);
CloudStyle style_violet (255.0, 0.0, 255.0, 8.0);

std::string model_filename_;
std::string scene_filename_;

//Algorithm params
bool show_keypoints_ (false);
bool show_correspondences_ (false);
bool use_cloud_resolution_ (false);
bool use_hough_ (true);
float model_ss_ (0.005f);
float scene_ss_ (0.008f);
float rf_rad_ (0.015f);
// float descr_rad_ (0.024f);
// float cg_size_ (0.015f);
// float descr_rad_ (0.018f);
// float cg_size_ (0.015f);
float descr_rad_ (0.018f);
float cg_size_ (0.015f);
float cg_thresh_ (5.0f);
int icp_max_iter_ (5);
float icp_corr_distance_ (0.005f);
float hv_resolution_ (0.005f);
float hv_occupancy_grid_resolution_ (0.01f);
float hv_clutter_reg_ (5.0f);
float hv_inlier_th_ (0.005f);
float hv_occlusion_th_ (0.01f);
float hv_rad_clutter_ (0.03f);
float hv_regularizer_ (3.0f);
float hv_rad_normals_ (0.05);
bool hv_detect_clutter_ (true);
// bool use_cloud_resolution_(true);
// bool use_hough_(true);
// float model_ss_(7.5f); //0.01f
// float scene_ss_(20.0f);//0.03f-20
// float rf_rad_(10.0f);  //0.015f-10
// float descr_rad_(15.0f);//0.02f -19 - 
// float cg_size_(10.0f);  //0.01f-10
// float cg_thresh_(5.0f);
 

class ObjectDetect {
  public:
    explicit ObjectDetect(ros::NodeHandle* nh, ros::NodeHandle* pnh);
    Eigen::Quaternionf ObjectRecognition(pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered, int i);
    geometry_msgs::Pose pose_oritention;
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
  std::cout << "*          Global Hypothese Verification Tutorial - Usage Guide          *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                          Show this help." << std::endl;
  std::cout << "     -k:                          Show keypoints." << std::endl;
  std::cout << "     --algorithm (Hough|GC):      Clustering algorithm used (default Hough)." << std::endl;
  std::cout << "     --model_ss val:              Model uniform sampling radius (default " << model_ss_ << ")" << std::endl;
  std::cout << "     --scene_ss val:              Scene uniform sampling radius (default " << scene_ss_ << ")" << std::endl;
  std::cout << "     --rf_rad val:                Reference frame radius (default " << rf_rad_ << ")" << std::endl;
  std::cout << "     --descr_rad val:             Descriptor radius (default " << descr_rad_ << ")" << std::endl;
  std::cout << "     --cg_size val:               Cluster size (default " << cg_size_ << ")" << std::endl;
  std::cout << "     --cg_thresh val:             Clustering threshold (default " << cg_thresh_ << ")" << std::endl << std::endl;
  std::cout << "     --icp_max_iter val:          ICP max iterations number (default " << icp_max_iter_ << ")" << std::endl;
  std::cout << "     --icp_corr_distance val:     ICP correspondence distance (default " << icp_corr_distance_ << ")" << std::endl << std::endl;
  std::cout << "     --hv_clutter_reg val:        Clutter Regularizer (default " << hv_clutter_reg_ << ")" << std::endl;
  std::cout << "     --hv_inlier_th val:          Inlier threshold (default " << hv_inlier_th_ << ")" << std::endl;
  std::cout << "     --hv_occlusion_th val:       Occlusion threshold (default " << hv_occlusion_th_ << ")" << std::endl;
  std::cout << "     --hv_rad_clutter val:        Clutter radius (default " << hv_rad_clutter_ << ")" << std::endl;
  std::cout << "     --hv_regularizer val:        Regularizer value (default " << hv_regularizer_ << ")" << std::endl;
  std::cout << "     --hv_rad_normals val:        Normals radius (default " << hv_rad_normals_ << ")" << std::endl;
  std::cout << "     --hv_detect_clutter val:     TRUE if clutter detect enabled (default " << hv_detect_clutter_ << ")" << std::endl << std::endl;
}

/**
 * Parses Command Line Arguments (Argc,Argv)
 * @param argc
 * @param argv
 */
void
parseCommandLine (int argc,
                  char *argv[])
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

  std::string used_algorithm;
  if (pcl::console::parse_argument (argc, argv, "--algorithm", used_algorithm) != -1)
  {
    if (used_algorithm.compare ("Hough") == 0)
    {
      use_hough_ = true;
    }
    else if (used_algorithm.compare ("GC") == 0)
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
  pcl::console::parse_argument (argc, argv, "--icp_max_iter", icp_max_iter_);
  pcl::console::parse_argument (argc, argv, "--icp_corr_distance", icp_corr_distance_);
  pcl::console::parse_argument (argc, argv, "--hv_clutter_reg", hv_clutter_reg_);
  pcl::console::parse_argument (argc, argv, "--hv_inlier_th", hv_inlier_th_);
  pcl::console::parse_argument (argc, argv, "--hv_occlusion_th", hv_occlusion_th_);
  pcl::console::parse_argument (argc, argv, "--hv_rad_clutter", hv_rad_clutter_);
  pcl::console::parse_argument (argc, argv, "--hv_regularizer", hv_regularizer_);
  pcl::console::parse_argument (argc, argv, "--hv_rad_normals", hv_rad_normals_);
  pcl::console::parse_argument (argc, argv, "--hv_detect_clutter", hv_detect_clutter_);
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

Eigen::Quaternionf ObjectDetect::ObjectRecognition(pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered, int i) {

  pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
  // pcl::PointCloud<PointType>::Ptr xyzCloudPtrFiltered1 (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
  pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

  //
  //  Load clouds
  //

  // if (pcl::io::loadPCDFile ("/home/iqr/catkin_ws/src/kinova_unit_ros/kinova_unit_app/pcd/sense_watson.pcd", *xyzCloudPtrFiltered) < 0)
  // {
  //   std::cout << "Error loading scene cloud." << std::endl;
  //   // showHelp (argv[0]);
  //   return;
  // }

  // if (pcl::io::loadPCDFile ("/home/iqr/catkin_ws/src/kinova_unit_ros/kinova_unit_app/pcd/object_filter.pcd", *model) < 0)
  // {
  //   std::cout << "Error loading scene cloud." << std::endl;
  //   // showHelp (argv[0]);
  //   // return 0;
  // }
  std::string filename = "/home/iqr/catkin_ws/src/kinova_unit_ros/kinova_unit_app/pcd/object" + std::to_string(i) + ".pcd";
  if (pcl::io::loadPCDFile (filename, *model) < 0)
  {
    std::cout << "Error loading scene cloud." << std::endl;
    // showHelp (argv[0]);
    // return 0;
  }

  /**
   * Compute Normals
   */
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch (10);
  norm_est.setInputCloud (model);
  norm_est.compute (*model_normals);

  norm_est.setInputCloud (xyzCloudPtrFiltered);
  norm_est.compute (*scene_normals);

  /**
   *  Downsample Clouds to Extract keypoints
   */
  pcl::UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud (model);
  uniform_sampling.setRadiusSearch (model_ss_);
  uniform_sampling.filter (*model_keypoints);
  std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

  uniform_sampling.setInputCloud (xyzCloudPtrFiltered);
  uniform_sampling.setRadiusSearch (scene_ss_);
  uniform_sampling.filter (*scene_keypoints);
  std::cout << "Scene total points: " << xyzCloudPtrFiltered->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;

  /**
   *  Compute Descriptor for keypoints
   */
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch (descr_rad_);

  descr_est.setInputCloud (model_keypoints);
  descr_est.setInputNormals (model_normals);
  descr_est.setSearchSurface (model);
  descr_est.compute (*model_descriptors);

  descr_est.setInputCloud (scene_keypoints);
  descr_est.setInputNormals (scene_normals);
  descr_est.setSearchSurface (xyzCloudPtrFiltered);
  descr_est.compute (*scene_descriptors);

  /**
   *  Find Model-Scene Correspondences with KdTree
   */
  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud (model_descriptors);
  std::vector<int> model_good_keypoints_indices;
  std::vector<int> scene_good_keypoints_indices;

  for (std::size_t i = 0; i < scene_descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
    if (!std::isfinite (scene_descriptors->at (i).descriptor[0]))  //skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back (corr);
      model_good_keypoints_indices.push_back (corr.index_query);
      scene_good_keypoints_indices.push_back (corr.index_match);
    }
  }
  pcl::PointCloud<PointType>::Ptr model_good_kp (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene_good_kp (new pcl::PointCloud<PointType> ());
  pcl::copyPointCloud (*model_keypoints, model_good_keypoints_indices, *model_good_kp);
  pcl::copyPointCloud (*scene_keypoints, scene_good_keypoints_indices, *scene_good_kp);

  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

  /**
   *  Clustering
   */
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector < pcl::Correspondences > clustered_corrs;

  if (use_hough_)
  {
    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);

    rf_est.setInputCloud (model_keypoints);
    rf_est.setInputNormals (model_normals);
    rf_est.setSearchSurface (model);
    rf_est.compute (*model_rf);

    rf_est.setInputCloud (scene_keypoints);
    rf_est.setInputNormals (scene_normals);
    rf_est.setSearchSurface (xyzCloudPtrFiltered);
    rf_est.compute (*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (cg_size_);
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (model_keypoints);
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (scene_keypoints);
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);

    // clusterer.cluster (clustered_corrs);
    clusterer.recognize (rototranslations, clustered_corrs);
  }
  else
  {
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    gc_clusterer.setGCSize (cg_size_);
    gc_clusterer.setGCThreshold (cg_thresh_);

    gc_clusterer.setInputCloud (model_keypoints);
    gc_clusterer.setSceneCloud (scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    gc_clusterer.recognize (rototranslations, clustered_corrs);
  }

  /**
   * Stop if no instances
   */
  if (rototranslations.size () <= 0)
  {
    std::cout << "*** No instances found! ***" << std::endl;
  }
  else
  {
    std::cout << "Recognized Instances: " << rototranslations.size () << std::endl << std::endl;

    /**
     * Generates clouds for each instances found 
     */
    std::vector<pcl::PointCloud<PointType>::ConstPtr> instances;

    for (std::size_t i = 0; i < rototranslations.size (); ++i)
    {
      pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
      pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);
      instances.push_back (rotated_model);
    }

    /**
     * ICP
     */
    std::vector<pcl::PointCloud<PointType>::ConstPtr> registered_instances;
    if (true)
    {
      std::cout << "--- ICP ---------" << std::endl;

      for (std::size_t i = 0; i < rototranslations.size (); ++i)
      {
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaximumIterations (icp_max_iter_);
        icp.setMaxCorrespondenceDistance (icp_corr_distance_);
        icp.setInputTarget (xyzCloudPtrFiltered);
        icp.setInputSource (instances[i]);
        pcl::PointCloud<PointType>::Ptr registered (new pcl::PointCloud<PointType>);
        icp.align (*registered);
        registered_instances.push_back (registered);
        std::cout << "Instance " << i << " ";
        if (icp.hasConverged ())
        {
          std::cout << "Aligned!" << std::endl;
        }
        else
        {
          std::cout << "Not Aligned!" << std::endl;
        }
      }

      std::cout << "-----------------" << std::endl << std::endl;
    }

    /**
     * Hypothesis Verification
     */
    std::cout << "--- Hypotheses Verification ---" << std::endl;
    std::vector<bool> hypotheses_mask;  // Mask Vector to identify positive hypotheses

    pcl::GlobalHypothesesVerification<PointType, PointType> GoHv;

    GoHv.setSceneCloud (xyzCloudPtrFiltered);  // Scene Cloud
    GoHv.addModels (registered_instances, true);  //Models to verify
    GoHv.setResolution (hv_resolution_);
    // GoHv.setResolutionOccupancyGrid (hv_occupancy_grid_resolution_);
    GoHv.setInlierThreshold (hv_inlier_th_);
    GoHv.setOcclusionThreshold (hv_occlusion_th_);
    GoHv.setRegularizer (hv_regularizer_);
    GoHv.setRadiusClutter (hv_rad_clutter_);
    GoHv.setClutterRegularizer (hv_clutter_reg_);
    GoHv.setDetectClutter (hv_detect_clutter_);
    GoHv.setRadiusNormals (hv_rad_normals_);

    GoHv.verify ();
    GoHv.getMask (hypotheses_mask);  // i-element TRUE if hvModels[i] verifies hypotheses
    Eigen::Quaternionf q_bad;
    std::cout << "Model instances found: " << rototranslations.size () << std::endl;
    for (int i = 0; i < hypotheses_mask.size (); i++)
    {
      if (hypotheses_mask[i])
      {
        std::cout << "Instance " << i << " is GOOD! <---" << std::endl;

        std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
        std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

        // Print the rotation matrix and translation vector
        Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
        Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

        printf ("\n");
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
        printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
        printf ("\n");
        printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

        Eigen::Quaternionf q(rotation);
        // cout<<"q=\n"<<q.coeffs()<<endl<<endl;        

        i = hypotheses_mask.size ();
        return q;
      }
      else
      {
        std::cout << "Instance " << i << " is bad!" << std::endl;
        // if (i == hypotheses_mask.size())
        // {
        //   q_bad.w() = 2.0;
        //   return q_bad;        
        // }
      }
    }
    // return q_bad;    
  }

  /**
   *  Visualization
   */
  // pcl::visualization::PCLVisualizer viewer ("Hypotheses Verification");
  // viewer.addPointCloud (xyzCloudPtrFiltered, "scene_cloud");

  // pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
  // pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

  // pcl::PointCloud<PointType>::Ptr off_model_good_kp (new pcl::PointCloud<PointType> ());
  // pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));
  // pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));
  // pcl::transformPointCloud (*model_good_kp, *off_model_good_kp, Eigen::Vector3f (-1, 0, 0), Eigen::Quaternionf (1, 0, 0, 0));

  // if (show_keypoints_)
  // {
  //   CloudStyle modelStyle = style_white;
  //   pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, modelStyle.r, modelStyle.g, modelStyle.b);
  //   viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
  //   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, modelStyle.size, "off_scene_model");
  // }

  // if (show_keypoints_)
  // {
  //   CloudStyle goodKeypointStyle = style_violet;
  //   pcl::visualization::PointCloudColorHandlerCustom<PointType> model_good_keypoints_color_handler (off_model_good_kp, goodKeypointStyle.r, goodKeypointStyle.g,
  //                                                                                                   goodKeypointStyle.b);
  //   viewer.addPointCloud (off_model_good_kp, model_good_keypoints_color_handler, "model_good_keypoints");
  //   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, goodKeypointStyle.size, "model_good_keypoints");

  //   pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_good_keypoints_color_handler (scene_good_kp, goodKeypointStyle.r, goodKeypointStyle.g,
  //                                                                                                   goodKeypointStyle.b);
  //   viewer.addPointCloud (scene_good_kp, scene_good_keypoints_color_handler, "scene_good_keypoints");
  //   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, goodKeypointStyle.size, "scene_good_keypoints");
  // }

  // for (std::size_t i = 0; i < instances.size (); ++i)
  // {
  //   std::stringstream ss_instance;
  //   ss_instance << "instance_" << i;

  //   CloudStyle clusterStyle = style_red;
  //   pcl::visualization::PointCloudColorHandlerCustom<PointType> instance_color_handler (instances[i], clusterStyle.r, clusterStyle.g, clusterStyle.b);
  //   viewer.addPointCloud (instances[i], instance_color_handler, ss_instance.str ());
  //   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, clusterStyle.size, ss_instance.str ());

  //   CloudStyle registeredStyles = hypotheses_mask[i] ? style_green : style_cyan;
  //   ss_instance << "_registered" << std::endl;
  //   pcl::visualization::PointCloudColorHandlerCustom<PointType> registered_instance_color_handler (registered_instances[i], registeredStyles.r,
  //                                                                                                  registeredStyles.g, registeredStyles.b);
  //   viewer.addPointCloud (registered_instances[i], registered_instance_color_handler, ss_instance.str ());
  //   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, registeredStyles.size, ss_instance.str ());
  // }

  // while (!viewer.wasStopped ())
  // {
  //   viewer.spinOnce ();
  // }

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
  // sor.setInputCloud (cloudPtr);
  // // sor.setLeafSize (0.005, 0.005, 0.005);
  // sor.setLeafSize (0.01, 0.01, 0.01);
  // sor.filter (*cloudFilteredPtr);

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
  pass_x.setFilterLimits (-0.2, 0.4);
  pass_x.filter (*xyzCloudPtrFiltered);

  pass_y.setInputCloud (xyzCloudPtrFiltered);
  pass_y.setFilterFieldName ("y");
  pass_y.setFilterLimits (-0.33, 0);
  pass_y.filter (*xyzCloudPtrFiltered);

  pass_z.setInputCloud (xyzCloudPtrFiltered);
  pass_z.setFilterFieldName ("z");
  pass_z.setFilterLimits (0.8, 1.015);
  pass_z.filter (*xyzCloudPtrFiltered);

  // object recognition
  // ObjectDetect obj;
  // std::thread recognition(&ObjectDetect::ObjectRecognition, &obj,*pose_oritention);

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
  seg1.setDistanceThreshold (0.022);

  seg1.setInputCloud (xyzCloudPtrFiltered);
  seg1.segment (*inliers, *coefficients);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  //extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*xyzCloudPtrRansacFiltered);

  // ObjectRecognition(xyzCloudPtrRansacFiltered);

  pcl::toPCLPointCloud2( *xyzCloudPtrRansacFiltered ,debugPCL);
  pcl_conversions::fromPCL(debugPCL, debug_output);
  debug_pointcloud_pub_.publish(debug_output);

  // perform euclidean cluster segmentation to seporate individual objects

  if(xyzCloudPtrFiltered->empty() == true) {
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
  ROS_INFO("%d", cluster_indices.size());
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
    ROS_INFO("%f %f %f", maxPoint[0]-minPoint[0], maxPoint[1]-minPoint[1], maxPoint[2]-minPoint[2]);

    object_visual_markers_pub_->publishAxis(top_pose);
    object_visual_markers_pub_->publishWireframeCuboid(centroid_pose, maxPoint[0]-minPoint[0], maxPoint[1]-minPoint[1], maxPoint[2]-minPoint[2], rviz_visual_tools::GREEN);
    
    int x,y,z = -1;
    bool finded = false;
    Eigen::Quaternionf q;
    // q = ObjectRecognition(clusterPtr, 0);
    ROS_INFO("start");
    // while(!finded) {
    for (int i = 0; i < 3; ++i)
    { 
      // if (i != x && i != y)
      // {
      //   /* code */
      // }
      q = ObjectRecognition(clusterPtr, i);
      // if (q.w() != 2)
      if (q.w() != 0)
      {
        // finded = true;
        i = 3;
      }
    }
    // }
    ROS_INFO("find");
    geometry_msgs::Pose pose;
    pose.position.x = top_pose.translation()[0];
    pose.position.y = top_pose.translation()[1];
    pose.position.z = top_pose.translation()[2];

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    if (q.w() != 0)
    {
      pose_array.poses.push_back(pose);
    }
    // convert to pcl::PCLPointCloud2
    pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);

    // Convert to ROS data type
    pcl_conversions::fromPCL(outputPCL, output);

  }
  // for (int i = 0; i < sizeof(pose_array); ++i)
  // {
    // pose_array[i].poses.orientation.x = ;
    // pose_array[i].poses.orientation.y = ;
    // pose_array[i].poses.orientation.z = ;
    // pose_array[i].poses.orientation.w = ;
  // }


  // pcl::toPCLPointCloud2(*cluster_indices ,debugPCL);
  // pcl_conversions::fromPCL(debugPCL, debug_output);
  // debug_pointcloud_pub_.publish(debug_output);



  pose_array.header.stamp = ros::Time::now();
  pose_array.header.frame_id = point_cloud_frame_;

  if(pose_array.poses.size() > 0) {
    latest_object_pose_.header.stamp = pose_array.header.stamp;
    latest_object_pose_.header.frame_id = pose_array.header.frame_id;
    latest_object_pose_.pose = pose_array.poses[0];
    object_pub_.publish(pose_array);
  }


  // object_visual_markers_pub_->trigger();
}


int main (int argc, char** argv) {
  ros::init (argc, argv, "object_detect_node");

  ros::NodeHandle nh, pnh("~");
  ObjectDetect object_detect(&nh, &pnh);
  // parseCommandLine (argc, argv);
  ros::spin();
}
