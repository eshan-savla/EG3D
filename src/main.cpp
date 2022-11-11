//
// Created by eshan on 07.11.22.
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include "RawCloud.h"
#include "EdgePoints.h"


int main(int argc, const char * argv[]) {
    RawCloud raw_input("../data/table_scene_lms400.pcd");
//    RawCloud raw_input(true, 1000);
    raw_input.VoxelDownSample(0.005f);
    std::cout << "Points after down sample: " << raw_input.GetCount() << std::endl;
    pcl::PointCloud<pcl::PointXYZ> cl = *raw_input.GetCloud();
    std::vector<int> edge_points;
    pcl::StopWatch stpw;
    std::cout << "Beginning edge point search" << std::endl;
    stpw.reset();
    EdgePoints edges = raw_input.FindEdgePoints(200, M_PI_2, edge_points, 0.01);
    double duration = stpw.getTimeSeconds();
    std::cout << "Processing duration: " << duration << std::endl;
    edges.Save("../data/edge_points.pcd");
    std::cout << "Segmenting edges" << std::endl;
    stpw.reset();
    edges.SegmentEdges(15, 15, 3.0/180.0 * M_PI, 1.0);
    duration = stpw.getTimeSeconds();
    std::cout << "Segmenting duration: " << duration << std::endl;
    //TODO:Consider visualisation implementation
    return 0;
}