//
// Created by eshan on 07.11.22.
//
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "RawCloud.h"


int main(int argc, const char * argv[]) {
    RawCloud raw_input("../data/Blech.pcd");
    raw_input.FindEdgePoints(100, 90, 0.01, 1, true);
    //TODO:Consider visualisation implementation
    return 0;
}