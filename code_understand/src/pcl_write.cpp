#include<ros/ros.h>//提供了ROS系统的基本功能，如节点（Node）的创建、发布者（Publisher）和订阅者（Subscriber）的定义等。
#include<pcl/point_cloud.h>//包含了点云（Point Cloud）数据结构的定义和相关操作，它位于PCL库的命名空间下。
#include<pcl_conversions/pcl_conversions.h>//提供了PCL与ROS之间的数据转换功能，用于将PCL中的数据类型转换为ROS中的数据类型，或者反之。
#include<sensor_msgs/PointCloud2.h>//定义了ROS中用于表示点云数据的消息类型，它位于sensor_msgs命名空间下。
#include<pcl/io/pcd_io.h>//提供了用于读取和写入点云数据到PCD文件的功能，位于PCL库的命名空间下
int i=0;
void cloudCB(const sensor_msgs::PointCloud2 &input)
{
  i++;
  pcl::PointCloud<pcl::PointXYZRGBA> cloud;//一个模板类，用于表示各种类型的点云数据。尖括号<>中的部分指定了点云中每个点的类型。
  pcl::fromROSMsg(input, cloud);//从ROS类型消息转为PCL类型消息
  if(i==1){
	  pcl::io::savePCDFileASCII ("/home/g/data/input_cloud.pcd", cloud);//保存pcd，路径改为自己的
    ROS_INFO("Save the pointcloud seccessful!");
}
 
}
int main(int argc, char **argv)

{
  ros::init (argc, argv, "pcl_write");
  ros::NodeHandle nh;
  ros::Subscriber bat_sub = nh.subscribe("filtered_RadiusOutlierRemoval", 10, cloudCB);
  //声明了一个ROS订阅者对象，名为 bat_sub,接收点云
  ros::spin();
  return 0;
}