#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//添加引用
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
 
ros::Publisher pub;
 
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_passed;
 
  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);
 
  // Perform the actual filtering
  pcl::PassThrough<pcl::PCLPointCloud2> pass;  //定义直通滤波对象
    // build the filter
  pass.setInputCloud (cloudPtr);
  pass.setFilterFieldName ("x"); //将在相机照射方向左右两侧x范围外的过滤
  pass.setFilterLimits (-0.4, 1.0);
    // apply filter
  pass.filter (cloud_passed);
 
  // Convert to ROS data type
  sensor_msgs::PointCloud2 cloud_pt;
  pcl_conversions::moveFromPCL(cloud_passed, cloud_pt);
 
  // Publish the data
  pub.publish (cloud_pt);
}
 
int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "PassThroughx");
  ros::NodeHandle nh;
 
  // Create a ROS subscriber for the input point cloud 输入
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/filtered_PassThroughz", 1, cloud_cb);
 
  // Create a ROS publisher for the output point cloud 输出
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_PassThrough", 1);
 
  // Spin
  ros::spin ();
}