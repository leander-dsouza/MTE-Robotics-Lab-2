//usr/bin/cc -o ${o=`mktemp`} "$0" && exec -a "$0" "$o" "$@"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "pcl/common/angles.h"
#include <math.h>
#include <signal.h>
#include <std_msgs/String.h>
#include <sstream>


using namespace std; 

ros::Publisher pub;
ros::Publisher chatter_pub;



long mag = 0.500;
long dir = 0.000; 

float mappy(float x, float in_min, float in_max, float out_min, float out_max)
{
  return float((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

// PASSTHROUGH -> VOXEL -> RANSAC -> CLUSTER -> BOUNDING BOX

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  
  //1)PASSTHROUGH
  // Container for input and output and its ptrs
  pcl::PointCloud<pcl::PointXYZRGB> *pass_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr passCloudPtr (pass_cloud);
  // create a pcl object to hold the passthrough filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *pass_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr passCloudFilteredPtr (pass_cloud_filtered);
  

  pcl::fromROSMsg (*input, *pass_cloud);


  

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (passCloudPtr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.1, 3.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*passCloudFilteredPtr);


  //2) VOXEL
    // Container for original & filtered data
  // Container for input and output and its ptrs
  pcl::PointCloud<pcl::PointXYZRGB> *voxel_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelCloudFilteredPtr (voxel_cloud_filtered);


  // Perform voxel grid downsampling filtering
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (passCloudFilteredPtr);
  sor.setLeafSize (0.02f, 0.02f, 0.02f);
  sor.filter (*voxelCloudFilteredPtr);


  //3) RANSAC
    // create a pcl object to hold the ransac filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *ransac_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ransacCloudFilteredPtr (ransac_cloud_filtered);


  // perform ransac planar filtration to remove inliers
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.04);
  seg.setInputCloud (voxelCloudFilteredPtr);
  seg.segment (*inliers, *coefficients);


  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  extract.setInputCloud (voxelCloudFilteredPtr);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*ransacCloudFilteredPtr);





  //4) EUCLIDEAN CLUSTERING

  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (ransacCloudFilteredPtr);

  // create the extraction object for the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;

  ec.setClusterTolerance (0.05); 
  ec.setMinClusterSize (300);
  ec.setMaxClusterSize (10000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (ransacCloudFilteredPtr);
  ec.extract (cluster_indices);

  

 //iterators

  std::vector<pcl::PointIndices>::const_iterator it;
  std::vector<int>::const_iterator pit;

  pcl::PointCloud<pcl::PointXYZRGB> *euclidean_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr euclideanCloudFilteredPtr (euclidean_cloud_filtered);

  pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);

  
  for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {

    for(pit = it->indices.begin(); pit != it->indices.end(); pit++)
      clusterPtr->points.push_back(ransacCloudFilteredPtr->points[*pit]);
         
     
    clusterPtr->width = clusterPtr->points.size ();
    clusterPtr->height = 1;
    clusterPtr->is_dense = true; 
    *euclideanCloudFilteredPtr += *clusterPtr;
 
  }


  Eigen::Vector4f centroid;
  geometry_msgs::Pose pose;
  Eigen::Vector4f min;
  Eigen::Vector4f max;

  pcl::compute3DCentroid (*euclideanCloudFilteredPtr, centroid);
  pcl::getMinMax3D (*euclideanCloudFilteredPtr, min, max);

  pose.position.x = centroid[0];
  pose.position.y = centroid[1];
  pose.position.z = centroid[2];

  

  // convert to rosmsg and publish:
  sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2 ());
  pcl::toROSMsg(*euclideanCloudFilteredPtr, *output);
  output->header.frame_id = "rs200_camera_rviz";
  output->header.stamp = ros::Time::now();

    
  pub.publish(output);

  ///////////////////////////////////////////////////////////////////////////////////

  centroid[2] = round( centroid[2] * 1000.0 ) / 1000.0;
  centroid[0] = round( centroid[0] * 1000.0 ) / 1000.0;
  
  if (centroid[0] < -1.5)
      centroid[0] = -1.500;
  if (centroid[0] > 1.5)
      centroid[0] = 1.500;


  if (centroid[0] < 0)
    {ROS_INFO("RIGHT");
    centroid[0] = mappy(centroid[0], -1.500, 0.000, 0.000, 90.000);}

  else if (centroid[0] > 0)
    {ROS_INFO("LEFT");
    centroid[0] = mappy(centroid[0], 0.000, 1.500, -90.000, 0.000);}
  else
    ROS_INFO("FORWARD");

  centroid[2] = mappy(centroid[2], 0.200, 3.000, 0.500, 1.000);
  
  
  std_msgs::String msg;
  std::stringstream ss;
  ss<<centroid[2]<<","<<centroid[0]<<"\0";
  msg.data = ss.str();
  chatter_pub.publish(msg);


  centroid[2] = 3.000;
  centroid[0] = 0.000;


  



  

}
 
int
main (int argc, char** argv)
{
  
  
  // Initialize ROS
  ros::init (argc, argv, "pcl_processing_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/r200/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_pcl", 1);
  chatter_pub = nh.advertise<std_msgs::String>("vector_prop", 1000);

  // Spin
  ros::spin ();
}
