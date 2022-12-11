//
// Created by chl-es on 10.12.22.
//

#include "Region2D.h"

Region2D::Region2D(const pcl::PointXYZ &mid_point1, const pcl::PointXYZ &mid_point2, double width) {
    this->width = width;
    depth = mid_point2.y - mid_point1.y;
    pcl::PointXYZ P1(mid_point1.x - (width/2.0), mid_point1.y, 0),
        P4(P1.x + this->width, P1.y + depth, 0);
    this->P1 = P1;
    this->P4 = P4;

}

bool Region2D::ChechIfPointInRegion(const pcl::PointXYZ point) {
    if ((point.x >= P1.x && point.x <= P4.x) && (point.y >= P1.y && point.y <= P4.y))
        return true;
    else
        return false;
}