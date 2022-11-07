//
// Created by eshan on 07.11.22.
//

#ifndef EG3D_RAWCLOUD_H
#define EG3D_RAWCLOUD_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>

class RawCloud {
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud;
    int count_before, count_after;
    float MeanK, StddevMulThresh, RadiusSearch, MinNeighbours;
    std::string file_path;

    void ReadCloud(const std::string &file_path);

public:
    RawCloud(const std::string &file_path);
};


#endif //EG3D_RAWCLOUD_H
