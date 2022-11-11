//
// Created by eshan on 11.11.22.
//

#include "EdgePoints.h"

EdgePoints::EdgePoints(const std::vector<int> &edge_indices, const pcl::PointCloud<pcl::PointXYZ>::Ptr& parent_cloud) :
edge_points(new pcl::PointCloud<pcl::PointXYZ>), edge_normals(new pcl::PointCloud<pcl::Normal>), tree(new pcl::search::KdTree<pcl::PointXYZ>) {
    pcl::copyPointCloud(*parent_cloud, edge_indices, *edge_points);
}

void EdgePoints::Save(const std::string& path) {
    pcl::io::savePCDFileASCII(path, *edge_points);
}

void EdgePoints::SegmentEdges(const int &neighbours_K1, const int &neighbours_K2, const float &smoothness_thresh,
                              const float &curvature_thresh) {
    unsigned int cloud_size = GetCount();
    int min_cluster_size = neighbours_K1;
    int max_cluster_size = int(cloud_size) * 2/3;

    EstimateNormals(neighbours_K1);
    pcl::IndicesPtr indices (new std::vector<int>);
    pcl::removeNaNFromPointCloud(*edge_points, *indices);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> region_grower;
    region_grower.setMinClusterSize(min_cluster_size);
    region_grower.setMaxClusterSize(max_cluster_size);
    region_grower.setSearchMethod(tree);
    region_grower.setNumberOfNeighbours(neighbours_K2);
    region_grower.setInputCloud(edge_points);
    region_grower.setIndices(indices);
    region_grower.setInputNormals(edge_normals);
    region_grower.setSmoothnessThreshold(smoothness_thresh);
    region_grower.setCurvatureThreshold(curvature_thresh);
    region_grower.extract(clusters);

    std::cout << "finished extraction" << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_coloured = region_grower.getColoredCloud();
    pcl::io::savePCDFileASCII("../data/segments.pcd", *cloud_coloured);
/*    pcl::visualization::CloudViewer viewer("Cluster Viewer");
    viewer.showCloud(cloud_coloured);
    while (!viewer.wasStopped())
    {
    }*/
}

void EdgePoints::EstimateNormals(const int neighbours_K) {
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(edge_points);
    normal_estimator.setKSearch(neighbours_K);
    normal_estimator.compute(*edge_normals);
}

unsigned int EdgePoints::GetCount() {
    return edge_points->width * edge_points->height;
}