//
// Created by eshan on 2/1/23.
//

#ifndef EG3D_SENSORINFO_H
#define EG3D_SENSORINFO_H
#include <Eigen/Dense>
struct SensorSpecs {
    float depth, height, width;
};

struct SensorCoords {
    Eigen::Vector3f x_axis, y_axis, z_axis, sensor_position;
};

#endif //EG3D_SENSORINFO_H
