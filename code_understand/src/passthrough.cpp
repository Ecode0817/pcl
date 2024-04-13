#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>//包含了 PCL 中定义的点云数据结构的头文件。点云数据结构是用来表示和处理三维点云数据的基本数据结构
#include <pcl/point_types.h>//包含了 PCL 中定义的点类型的头文件。PCL 提供了许多预定义的点类型，如 XYZ、XYZRGB、XYZRGBA 等
//添加引用
#include <pcl/PCLPointCloud2.h>//处理任意类型的点云数据，而不需要事先知道点云的具体类型。
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
 
ros::Publisher pub;
 
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
//定以 _cb 结尾的函数名通常表示为回调函数,智能指针可以有效地管理指向消息数据的内存，并避免不必要的内存拷贝。
//const：这表示 cloud_msg 是一个指向常量对象的指针，即指针本身不可变,确保在回调函数中不会修改接收到的消息数据。
{
  // 原始数据和过滤数据的容器
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_passed;
 
  // 转换为 PCL 数据类型
  pcl_conversions::toPCL(*cloud_msg, *cloud);
 
  // 执行实际过滤
  pcl::PassThrough<pcl::PCLPointCloud2> pass;  //定义直通滤波对象
    // build the filter
  pass.setInputCloud (cloudPtr);
  pass.setFilterFieldName ("z"); //将在相机照射方向z范围外的过滤
  pass.setFilterLimits (0, 0.5);
    // apply filter
  pass.filter (cloud_passed);
 
  // 将pcl点云数据转换为ros数据，并进行存储 
  sensor_msgs::PointCloud2 cloud_pt;
  pcl_conversions::moveFromPCL(cloud_passed, cloud_pt);
 
  // Publish the data
  pub.publish (cloud_pt);
}
 
int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "PassThrough");
  ros::NodeHandle nh;
 
  // Create a ROS subscriber for the input point cloud 输入
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/cloud_input", 1, cloud_cb);
  //ROS 订阅者将订阅 sensor_msgs::PointCloud2 类型的消息，即点云数据的消息类型。
 
  // Create a ROS publisher for the output point cloud 输出
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_PassThroughz", 1);
 
  // Spin
  ros::spin ();
}