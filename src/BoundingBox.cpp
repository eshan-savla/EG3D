//
// Created by chl-es on 10.12.22.
//

#include <pcl/point_cloud.h>
#include "BoundingBox.h"

BBCloudSection::BBCloudSection(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_section,
                               const Eigen::Vector3f &dir_vec,
                               const float width) {
    this->dir_vec = dir_vec;
    this->width = width;
    InitPoints(cloud_section);
}

void BBCloudSection::InitPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_section) {
    Eigen::Vector4f pca_centroid;
    pcl::compute3DCentroid(*cloud_section, pca_centroid);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pca_projection (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud_section);
    pca.project(*cloud_section, *pca_projection);
    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();

//    pcl::io::savePCDFileASCII("/home/eshan/TestEG3D/src/testeg3d/data/projected_cloud.pcd", *pca_projection);
    pcl::PointXYZ min, max;
    pcl::getMinMax3D(*pca_projection, min, max);
    const Eigen::Vector3f diagonal = 0.5f*(max.getVector3fMap() + min.getVector3fMap());

    const Eigen::Quaternionf b_box_quaternion(eigen_vectors);
    const Eigen::Vector3f b_box_translation = eigen_vectors * diagonal + pca_centroid.head<3>();
    const Eigen::Quaternionf b_box_quat_norm = b_box_quaternion.normalized();
    const Eigen::Matrix3f rotation_matrix = b_box_quat_norm.toRotationMatrix();

    Eigen::Vector3f depth_vec(max.x - min.x, 0.0, 0.0), width_vec(0.0, max.y - min.y, 0.0), height_vec(0.0, 0.0, max.z - min.z);
    Eigen::Vector3f P1_vec = b_box_translation + (rotation_matrix * min.getVector3fMap()), P8_vec = b_box_translation + (rotation_matrix * max.getVector3fMap()),
            P2_vec = b_box_translation + (rotation_matrix * (min.getVector3fMap() + depth_vec)), P3_vec = b_box_translation + (rotation_matrix * (min.getVector3fMap() + width_vec)),
            P4_vec = b_box_translation + (rotation_matrix * (min.getVector3fMap() + depth_vec + width_vec));
    depth = P8.x - P1.x; width = P8.y - P1.y; height = P8.z - P1.z;

    P1 = {P1_vec.x(), P1_vec.y(), P1_vec.z()};
    P2 = {P2_vec.x(), P2_vec.y(), P2_vec.z()};
    P3 = {P3_vec.x(), P3_vec.y(), P3_vec.z()};
    P4 = {P4_vec.x(), P4_vec.y(), P4_vec.z()};
    P5 = {P1_vec.x(), P1_vec.y(), P1_vec.z() + height};
    P6 = {P2_vec.x(), P2_vec.y(), P2_vec.z() + height};
    P7 = {P3_vec.x(), P3_vec.y(), P3_vec.z() + height};
    P8 = {P8_vec.x(), P8_vec.y(), P8_vec.z()};
}

void BBCloudSection::CreateBorderRegions() {
    Region2D region_last(P7, P8, dir_vec, width);
    region_last.GetPoints(last_x, last_y);
    Region2D region_first(P5, P6, dir_vec, width);
    region_last.GetPoints(first_x, first_y);
}

void BBCloudSection::GetPoints(pcl::PointXYZ *points) {
    pcl::PointXYZ points_ [8] = {P1, P2, P3, P4, P5, P6, P7, P8};
    for (int i = 0; i < 8; i++)
        *(points + i) = points_[i];
}

bool BBCloudSection::CheckIfPointInFirstRegion(const pcl::PointXYZ &point) {
    if ((point.x >= first_x[0] && point.x <= first_x[1]) && (point.y >= first_y[0] && point.y <= first_y[1]))
        return true;
    else
        return false;
}

bool BBCloudSection::CheckIfPointInLastRegion(const pcl::PointXYZ &point) {
    if ((point.x >= last_x[0] && point.x <= last_x[1]) && (point.y >= last_y[0] && point.y <= last_y[1]))
        return true;
    else
        return false;
}


BBSensorPos::BBSensorPos(const SensorCoords &first, const SensorCoords &last, const SensorSpecs &specs,
                         const float width) {
    this->sensor_coords_first = first;
    this->sensor_coords_last = last;
    this->specs = specs;
    this->width = width;
    InitPoints();
}

void BBSensorPos::InitPoints() {
    Eigen::Vector3f width_vec, depth_vec, height_vec, P1_vec_first, P8_vec_first, P1_vec_last, P8_vec_last;
    width_vec = sensor_coords_first.y_axis;
    width_vec = (width / 2.0) * width_vec.normalized();
    depth_vec = sensor_coords_first.x_axis;
    depth_vec = (specs.depth / 2.0) * depth_vec.normalized();
    height_vec = sensor_coords_first.z_axis;
    height_vec = (1.5 * specs.height) * height_vec.normalized();

    Eigen::Vector3f pos = sensor_coords_first.sensor_position;

    P1_vec_first = sensor_coords_first.sensor_position - width_vec - depth_vec;
    P8_vec_first = sensor_coords_first.sensor_position + width_vec + depth_vec + height_vec;
    P1_first = {P1_vec_first[0], P1_vec_first[1], P1_vec_first[2]};
    P8_first = {P8_vec_first[0], P8_vec_first[1], P8_vec_first[2]};

    width_vec = (width/2) * sensor_coords_last.y_axis.normalized();
    depth_vec = specs.depth * sensor_coords_last.x_axis.normalized();
    height_vec = (1.5 * specs.height) * sensor_coords_last.z_axis.normalized();

    Eigen::Vector3f pos2 = sensor_coords_last.sensor_position;

    P1_vec_last = sensor_coords_last.sensor_position - width_vec - depth_vec;
    P8_vec_last = sensor_coords_last.sensor_position + width_vec + depth_vec + height_vec;
    P1_last = {P1_vec_last[0], P1_vec_last[1], P1_vec_last[2]};
    P8_last = {P8_vec_last[0], P8_vec_last[1], P8_vec_last[2]};
}

void BBSensorPos::GetPoints(pcl::PointXYZ *points) {
    pcl::PointXYZ points_[4] = {P1_first, P8_first, P1_last, P8_last};
    for (int i = 0; i < 4; ++i) {
        *(points + i) = points_[i];
    }
}

bool BBSensorPos::CheckIfPointInFirstRegion(const pcl::PointXYZ &point) {
    float xmin, xmax, ymin, ymax, zmin, zmax;
    if (P1_first.x < P8_first.x) {
        xmin = P1_first.x;
        xmax = P8_first.x;
    }
    else {
        xmin = P8_first.x;
        xmax = P1_first.x;
    }

    if (P1_first.y < P8_first.y) {
        ymin = P1_first.y;
        ymax = P8_first.y;
    }
    else {
        ymin = P8_first.y;
        ymax = P1_first.y;
    }

    if (P1_first.z < P8_first.z) {
        zmin = P1_first.z;
        zmax = P8_first.z;
    }
    else {
        zmin = P8_first.z;
        zmax = P1_first.z;
    }

    if ((point.x >= xmin && point.x <= xmax) && (point.y >= ymin && point.x <= ymax) && (point.z >= zmin && point.z <= zmax))
        return true;
    else
        return false;
}

bool BBSensorPos::CheckIfPointInLastRegion(const pcl::PointXYZ &point) {
    float xmin, xmax, ymin, ymax, zmin, zmax;
    if (P1_last.x < P8_last.x) {
        xmin = P1_last.x;
        xmax = P8_last.x;
    }
    else {
        xmin = P8_last.x;
        xmax = P1_last.x;
    }

    if (P1_last.y < P8_last.y) {
        ymin = P1_last.y;
        ymax = P8_last.y;
    }
    else {
        ymin = P8_last.y;
        ymax = P1_last.y;
    }

    if (P1_last.z < P8_last.z) {
        zmin = P1_last.z;
        zmax = P8_last.z;
    }
    else {
        zmin = P8_last.z;
        zmax = P1_last.z;
    }

    if ((point.x >= xmin && point.x <= xmax) && (point.y >= ymin && point.x <= ymax) && (point.z >= zmin && point.z <= zmax))
        return true;
    else
        return false;
}
