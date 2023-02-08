//
// Created by eshan on 07.11.22.
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include "RawCloud.h"
#include "EdgeCloud.h"


int main(int argc, const char * argv[]) {
    RawCloud raw_input;
    raw_input.ReadCloud("../data/Blech.pcd");
//    raw_input.GenerateCloud(1000);
    raw_input.VoxelDownSample(0.0001f);
    raw_input.StatOutlierRemoval(50,1.0);
    std::cout << "Points after down sample: " << raw_input.GetCount() << std::endl;
    std::vector<int> edge_points;
    pcl::StopWatch stpw;
    std::cout << "Beginning edge point search" << std::endl;
    stpw.reset();
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    *edge_cloud = raw_input.FindEdgePoints(200, M_PI_2, 0.01);
    EdgeCloud edges(edge_cloud);
    double duration = stpw.getTimeSeconds();
    std::cout << "Processing duration: " << duration << std::endl;
    edges.SaveCloud("../data/edge_points.pcd");
    std::cout << "Segmenting edges" << std::endl;
//    EdgeCloud edges;
//    edges.ReadCloud("../data/edge_points.pcd");
    stpw.reset();
    edges.SegmentEdges(15, 0.01, 10.0 / 180.0 * M_PI, true, false);
    duration = stpw.getTimeSeconds();
    std::cout << "Segmenting duration: " << duration << std::endl;
    edges.CreateColouredCloud("../data/segments.pcd");
    //TODO:Consider visualisation implementation
    return 0;
}