//
// Created by chl-es on 10.12.22.
//

#include "Region2D.h"

Region2D::Region2D(const pcl::PointXYZ &mid_point_bottom, const pcl::PointXYZ &mid_point_top, Eigen::Vector3f width_vec, double width) {
    Eigen::Vector3f mid_1_vec = mid_point_bottom.getVector3fMap();
    if(width_vec.norm() < 0)
        width_vec *= -1;
    width_vec.normalize();
    width_vec *= width / 2.0;
    depth_vec = mid_point_top.getVector3fMap() - mid_point_bottom.getVector3fMap();
    depth_vec = (depth_vec.norm()/4) * depth_vec.normalized();
    Eigen::Vector3f P1_vec(mid_1_vec - width_vec - depth_vec), P4_vec(mid_point_top.getVector3fMap() + width_vec + depth_vec);

    P1 = {P1_vec(0), P1_vec(1), P1_vec(2)};
    P4 = {P4_vec(0), P4_vec(1), P4_vec(2)};
    if(P1.x < P4.x) {
        xmin = P1.x;
        xmax = P4.x;
    }
    else {
        xmin = P4.x;
        xmax = P1.x;
    }

    if (P1.y < P4.y) {
        ymin = P1.y;
        ymax = P4.y;
    }
    else {
        ymin = P4.y;
        ymax = P1.y;
    }
}

bool Region2D::ChechIfPointInRegion(const pcl::PointXYZ point) {
    if ((point.x >= xmin && point.x <= xmax) && (point.y >= ymin && point.y <= ymax))
        return true;
    else
        return false;
}