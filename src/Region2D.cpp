//
// Created by chl-es on 10.12.22.
//

#include "Region2D.h"

Region2D::Region2D(const pcl::PointXYZ &mid_point1, const pcl::PointXYZ &mid_point2, float width) {
    this->width = width;
    depth = mid_point2.y - mid_point1.y;
    pcl::PointXYZ P1(mid_point1.x - (width/2.0), mid_point1.y, 0),
        P2(mid_point1.x - (width/2.0), mid_point2.y, 0),
        P3(mid_point1.x + (width / 2.0), mid_point1.y, 0),
        P4(mid_point1.x + (width / 2.0), mid_point2.y, 0);
    this->P1 = P1.getArray3fMap(); this->P2 = P2.getArray3fMap();
    this->P3 = P3.getArray3fMap(); this->P4 = P4.getArray3fMap();
    
}