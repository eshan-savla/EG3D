//
// Created by eshan on 07.11.22.
//
#include <iostream>
#include <fstream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include "RawCloud.h"
#include "EdgeCloud.h"


/////////////////////// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX //////////////////////////////////////////////

// Static variables for argument parsing
static std::string filename;
static int MeanK1, NeighboursK1, MeanK2, NeighboursK2;
static float StddevMulThresh1, dist_thresh1, angular_thresh1, StddevMulThresh2, dist_thresh2,
        angular_thresh2;
static double leaf_radius;
static bool SetStatOutRem, sort;

/////////////////////// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX //////////////////////////////////////////////

std::unordered_map<std::string, std::string> ArgParser(const std::string& config){
    std::unordered_map<std::string, std::string> args;
    std::ifstream config_file(config);
    std::stringstream buffer;
    buffer << config_file.rdbuf();
    std::istringstream data(buffer.str());
    std::string line;
        while(getline(data, line)) {
            std::istringstream is_line(line);
            std::string key;
            if (std::getline(is_line, key, '=')) {
                std::string value;
                if (std::getline(is_line, value))
                    args[key] = value;
            }
        }
    return args;
}

void AssignArgs(const std::unordered_map<std::string, std::string>& args){
    filename = args.at("filename");
    MeanK1 = std::stoi(args.at("MeanK1"));
    NeighboursK1 = std::stoi(args.at("NeighboursK1"));
    MeanK2 = std::stoi(args.at("MeanK2"));
    NeighboursK2 = std::stoi(args.at("NeighboursK2"));
    leaf_radius = std::stof(args.at("leaf_radius"));
    StddevMulThresh1 = std::stof(args.at("StddevMulThresh1"));
    dist_thresh1 = std::stof(args.at("StddevMulThresh1"));
    angular_thresh1 = M_PI * std::stof(args.at("angular_thresh1")) / 180.0;
    StddevMulThresh2 = std::stof(args.at("StddevMulThresh2"));
    dist_thresh2 = std::stof(args.at("dist_thresh2"));
    angular_thresh2 = M_PI * std::stof(args.at("angular_thresh2")) / 180.0;
    SetStatOutRem = std::stoi(args.at("SetStatOutRem"));
    sort = std::stoi(args.at("sort"));

}

int main(int argc, const char * argv[]) {
    std::unordered_map<std::string, std::string> args = ArgParser("../config");
    AssignArgs(args);
    RawCloud raw_input;
    raw_input.ReadCloud(filename);
//    raw_input.GenerateCloud(1000);
//    raw_input.UniformDownSample(leaf_radius);
//    raw_input.StatOutlierRemoval(MeanK1,StddevMulThresh1);
//    std::cout << "Points after down sample: " << raw_input.GetCount() << std::endl;
    std::vector<int> edge_points;
    pcl::StopWatch stpw;
    std::cout << "Beginning edge point search" << std::endl;
    stpw.reset();
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    *edge_cloud = raw_input.FindEdgePoints(NeighboursK1, angular_thresh1, dist_thresh1);
    EdgeCloud edges;
    double duration = stpw.getTimeSeconds();
//    edges.SetStatOutRem(SetStatOutRem, MeanK2, StddevMulThresh2);
    edges.AddPoints(edge_cloud);
    std::cout << "Processing duration: " << duration << std::endl;
    edges.SaveCloud("../data/edge_points.pcd");
    std::cout << "Segmenting edges" << std::endl;
//    EdgeCloud edges;
//    edges.ReadCloud("../data/edge_points.pcd");
    stpw.reset();
    edges.SegmentEdges(NeighboursK2, dist_thresh2, angular_thresh2, sort, false);
    duration = stpw.getTimeSeconds();
    std::cout << "Segmenting duration: " << duration << std::endl;
    edges.CreateColouredCloud("../data/segments.pcd");
    return 0;
}