//
// Created by chl-es on 10.12.22.
//

#ifndef EG3D_REGION2D_H
#define EG3D_REGION2D_H

#include <pcl/point_types.h>

class Region2D {
private:
    pcl::PointXYZ P1, P4;
    Eigen::Vector3f width_vec, depth_vec;
    float xmin, xmax, ymin, ymax;


public:
    Region2D(const pcl::PointXYZ& mid_point_bottom, const pcl::PointXYZ& mid_point_top, Eigen::Vector3f width_vec, double width);
    bool ChechIfPointInRegion(const pcl::PointXYZ point);
    void GetPoints(float* x_val, float* y_val);
};



#endif //EG3D_REGION2D_H
