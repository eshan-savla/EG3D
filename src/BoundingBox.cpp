//
// Created by chl-es on 10.12.22.
//

#include "BoundingBox.h"

BoundingBox::BoundingBox(const pcl::PointXYZ &min, const pcl::PointXYZ &max) {
    P1 = min; P8 = max;
    width = P8.y - P1.y;
    depth = P8.x - P1.x;
    height = P8.z - P1.z;
    InitPoints();
}

void BoundingBox::InitPoints() {
    pcl::PointXYZ P2(P1.x, P1.y + depth, P1.z),
        P3(P1.x + width, P1.y, P1.z),
        P4(P1.x + width, P1.y + depth, P1.z),
        P5(P8.x - width, P8.y - depth, P8.z),
        P6(P8.x - width, P8.y, P8.z),
        P7(P8.x, P8.y - depth, P8.z);
    this->P2 = P2; this->P3 = P3; this->P4 = P4;
    this->P5 = P5; this->P6 = P6; this->P7 = P7;
}

void BoundingBox::GetPoints(pcl::PointXYZ *points) {
    pcl::PointXYZ points_ [8] = {P1, P2, P3, P4, P5, P6, P7, P8};
    for (int i = 0; i < 8; i++)
        *(points + i) = points_[i];
    
}