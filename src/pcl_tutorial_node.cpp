#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/passthrough.h>

float z_min_filter_limit, z_max_filter_limit;

float distanceThreshold;

std::string filterFieldName;


ros::Publisher pub;
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_msg, *raw_cloud);

//New code will go here
  //pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  //sor.setInputCloud (raw_cloud);
  //sor.setLeafSize (0.05f, 0.05f, 0.05f);
  //sor.filter (*transformed_cloud);
  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (raw_cloud);
  pass.setFilterFieldName (filterFieldName);
  pass.setFilterLimits (z_min_filter_limit, z_max_filter_limit);
  pass.filter (*indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
  reg.setInputCloud (raw_cloud);
  reg.setIndices (indices);
  reg.setSearchMethod (tree);
  reg.setDistanceThreshold (distanceThreshold);
  reg.setPointColorThreshold (6);
  reg.setRegionColorThreshold (5);
  reg.setMinClusterSize (600);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  
  transformed_cloud = reg.getColoredCloud();

  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(*transformed_cloud, output_msg);

  output_msg.header.frame_id = cloud_msg->header.frame_id;
  output_msg.header.stamp = cloud_msg->header.stamp;

  pub.publish(output_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_tutorial_cloud");
  ros::NodeHandle n("~");
  ros::Subscriber sub =
      n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, cloud_cb);
  pub = n.advertise<sensor_msgs::PointCloud2>("output", 1);
  //ros::spin();

  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    n.param<float>("z_min_filter_limit", z_min_filter_limit, 0.0);
    n.param<float>("z_max_filter_limit", z_max_filter_limit, 1.0);
    n.param<float>("distanceThreshold", distanceThreshold, 10);
    n.param<std::string>("filterFieldName", filterFieldName, "z");

    ros::spinOnce();
    loop_rate.sleep();
  //GET PARAMETER HERE
  }

}