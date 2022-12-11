//
// Created by chl-es on 10.12.22.
//

#ifndef EG3D_BOUNDINGBOX_H
#define EG3D_BOUNDINGBOX_H

#include <pcl/point_types.h>
#include <pcl/point_representation.h>

class BoundingBox {
private:
    pcl::PointXYZ P1, P2, P3, P4, P5, P6, P7, P8;
    void InitPoints();
    float width, height, depth;

public:
    BoundingBox(const pcl::PointXYZ& min, const pcl::PointXYZ& max);
    pcl::PointXYZ* GetPoints();

};


#endif //EG3D_BOUNDINGBOX_H
