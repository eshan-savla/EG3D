//
// Created by chl-es on 10.12.22.
//

#ifndef EG3D_REGION2D_H
#define EG3D_REGION2D_H

#include <pcl/point_types.h>

class Region2D {
private:
    Eigen::Vector3f P1, P2, P3, P4;
    Eigen::Vector3f u, v;
    float width, depth;

public:
    Region2D(const pcl::PointXYZ& mid_point1, const pcl::PointXYZ& mid_point2, float width);
    bool ChechIfPointInRegion(const pcl::PointXYZ point_copy);
};



#endif //EG3D_REGION2D_H
