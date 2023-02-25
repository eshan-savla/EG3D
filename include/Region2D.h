//
// Created by chl-es on 10.12.22.
//

#ifndef EG3D_REGION2D_H
#define EG3D_REGION2D_H

#include <pcl/point_types.h>

/// @brief Class to create 2D Region around given line
class Region2D {
private:
    pcl::PointXYZ P1, P4;
    Eigen::Vector3f width_vec, depth_vec;
    float xmin, xmax, ymin, ymax;


public:
    /// @brief Constructor creates a 2D Region around line defined by arguments.
    ///
    /// This method utilizes 2 points to define a middle line and creates a two dimensional rectangle around it using a width vector for guidance.
    /// @param[in] mid_point_bottom 3D Point defining the bottom of the middle line
    /// @param[in] mid_point_top 3D Point defining the top of the middle line
    /// @param[in] width_vec Vector to guide the direction in which width is applied
    /// @param[in] width Defines the width of the rectangular region in the direction of the width vector
    Region2D(const pcl::PointXYZ& mid_point_bottom, const pcl::PointXYZ& mid_point_top, Eigen::Vector3f width_vec, double width);

    /// @brief Method to check if given point lies in the 2D region.
    /// @param[in] point Point to be checked
    /// @return Boolean value whether point lies in region.
    bool ChechIfPointInRegion(const pcl::PointXYZ point);

    /// @brief Getter to output corner points of the region
    /// @param[out] x_val Pointer pointing to vector of size 2 to store min & max x values
    /// @param[out] y_val Pointer pointing to vector of size 2 to store min & max y values
    void GetPoints(float* x_val, float* y_val);
};



#endif //EG3D_REGION2D_H
