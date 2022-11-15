//
// Created by eshan on 07.11.22.
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include "RawCloud.h"


int main(int argc, const char * argv[]) {
    RawCloud raw_input("../data/test_scan_1.pcd");
//    RawCloud raw_input(true, 1000);
    raw_input.VoxelDownSample(0.0001f);
    std::cout << "Points after down sample: " << raw_input.GetCount() << std::endl;
    pcl::PointCloud<pcl::PointXYZ> cl = *raw_input.GetCloud();
    std::vector<int> edge_points;
    pcl::StopWatch stpw;
    std::cout << "Beginning edge point search" << std::endl;
    stpw.reset();
    raw_input.FindEdgePoints(200, M_PI_2, edge_points, 0.01);
    double duration = stpw.getTimeSeconds();
    std::cout << "Processing duration: " << duration << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(cl, edge_points, *edge_cloud);
    pcl::io::savePCDFileASCII("../data/edge_points.pcd", *edge_cloud);
    //TODO:Consider visualisation implementation
    return 0;
}