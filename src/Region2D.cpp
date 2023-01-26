//
// Created by chl-es on 10.12.22.
//

#include "Region2D.h"

Region2D::Region2D(const pcl::PointXYZ &mid_point1, const pcl::PointXYZ &mid_point2, const pcl::PointXYZ &angle_ref, double width) {
    Eigen::Vector3f mid_1_vec = mid_point1.getVector3fMap(), angle_ref_vec = angle_ref.getVector3fMap();
    width_vec = mid_1_vec - angle_ref_vec;
    if(width_vec.norm() < 0)
        width_vec *= -1;
    width_vec.normalize();
    width_vec *= width / 2.0;
    depth_vec = mid_point2.getVector3fMap() - mid_point1.getVector3fMap();
    Eigen::Vector3f P1_vec(mid_1_vec - width_vec), P4_vec(mid_point2.getVector3fMap() + width_vec);

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