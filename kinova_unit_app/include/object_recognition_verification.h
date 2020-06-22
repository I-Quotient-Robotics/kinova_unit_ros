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
