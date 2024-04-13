#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char** argv) {
    // Check input arguments
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " input.pcd output.pcd" << std::endl;
        return -1;
    }

    // Load input file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s \n", argv[1]);
        return -1;
    }

    // Perform pass through filtering along X and Z directions
    // 右手笛卡尔坐标系，Z轴指向正前方
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.3, -0.16);
    pass.filter(*cloud);
    
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 0.4);
    pass.filter(*cloud);

    // pass.setInputCloud(cloud);
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(0, 0.4);
    // pass.filter(*cloud);

    // // Perform radius outlier removal
    // pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    // outrem.setInputCloud(cloud);
    // outrem.setRadiusSearch(0.8);
    // outrem.setMinNeighborsInRadius(5);
    // outrem.filter(*cloud);

    // Perform voxel grid downsampling
    // pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    // sor.setInputCloud(cloud);
    // sor.setLeafSize(0.001, 0.001, 0.001);
    // sor.filter(*cloud);

    // Save output file
    pcl::io::savePCDFileBinary(argv[2], *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " << argv[2] << std::endl;

    return 0;
}
