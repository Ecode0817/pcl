#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " input_cloud.pcd output_cable_cluster.pcd" << std::endl;
        return -1;
    }

    // Load input point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", argv[1]);
        return -1;
    }

    if (cloud->empty()) {
        std::cerr << "Input cloud is empty" << std::endl;
        return -1;
    }

    // Voxel Grid Downsampling
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.003, 0.003, 0.003);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    vg.filter(*cloud_downsampled);

    // Radius Outlier Removal
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
    ror.setInputCloud(cloud_downsampled);
    ror.setRadiusSearch(0.1);
    ror.setMinNeighborsInRadius(20);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    ror.filter(*cloud_filtered);

    // Euclidean Clustering
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.012);
    ec.setMinClusterSize(200);
    ec.setMaxClusterSize(20000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

	// Find the cluster with the fewest points (assuming it's the cable)
	int min_cluster_size = std::numeric_limits<int>::max(); // 初始化为一个较大的值
	int min_cluster_index = 0;
	for (size_t i = 0; i < cluster_indices.size(); ++i) {
		if (cluster_indices[i].indices.size() < min_cluster_size) {
			min_cluster_size = cluster_indices[i].indices.size();
			min_cluster_index = i;
		}
	}

	// Extract the cable cluster
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cable_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (std::vector<int>::const_iterator pit = cluster_indices[min_cluster_index].indices.begin();
		pit != cluster_indices[min_cluster_index].indices.end(); ++pit) {
		cable_cluster->push_back((*cloud_filtered)[*pit]);
}

    // Save cable cluster to file
    pcl::io::savePCDFileASCII(argv[2], *cable_cluster);

    return 0;
}
