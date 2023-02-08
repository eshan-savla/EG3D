//
// Created by chl-es on 10.12.22.
//

#include "Region2D.h"

Region2D::Region2D(const pcl::PointXYZ &mid_point_bottom, const pcl::PointXYZ &mid_point_top, Eigen::Vector3f width_vec, double width) {
    Eigen::Vector3f mid_1_vec = mid_point_bottom.getVector3fMap();
    this->width_vec = (width / 2.0) * width_vec.normalized();
    depth_vec = mid_point_top.getVector3fMap() - mid_point_bottom.getVector3fMap();
    depth_vec = (depth_vec.norm()/4) * depth_vec.normalized();
    float depth = depth_vec.norm();
    Eigen::Vector3f P1_vec(mid_1_vec - this->width_vec), P4_vec(mid_point_top.getVector3fMap() + this->width_vec);

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

void Region2D::GetPoints(float *x_val, float *y_val) {
    *x_val = xmin; *(x_val + 1) = xmax;
    *y_val = ymin; *(y_val + 1) = ymax;
}
