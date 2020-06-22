#include "kinova_unit_app/object_save.h"

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
  pass_x.setFilterLimits (-0.22, 0.42);
  pass_x.filter (*xyzCloudPtrFiltered);

  pass_y.setInputCloud (xyzCloudPtrFiltered);
  pass_y.setFilterFieldName ("y");
  pass_y.setFilterLimits (-0.43, 0);
  pass_y.filter (*xyzCloudPtrFiltered);

  pass_z.setInputCloud (xyzCloudPtrFiltered);
  pass_z.setFilterFieldName ("z");
  pass_z.setFilterLimits (0.85, 1.08);
  pass_z.filter (*xyzCloudPtrFiltered);

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
  seg1.setDistanceThreshold (0.018);

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

    writer.write ("/home/iqr/object2.pcd", *xyzCloudPtrRansacFiltered, false);
    ROS_INFO("write");
  }
  writeornot += 1;
}

