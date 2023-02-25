//
// Created by eshan on 2/1/23.
//

#ifndef EG3D_SENSORINFO_H
#define EG3D_SENSORINFO_H
#include <Eigen/Dense>
/// @brief Struct containing information on scan range of the sensor.
struct SensorSpecs {
    float depth, ///< Maximum depth (x axis) sensor can scan
     height, ///< Maximum height (z axis) sensor can scan
     width; ///< Maximum width (y axis) sensor can scan
};

/// @brief Struct containing information on sensor coordinate system and position
struct SensorCoords {
    Eigen::Vector3f x_axis, ///< Eigen vector representing x axis of sensor coordinate system in world coordinate system
    y_axis, ///< Eigen vector representing y axis of sensor coordinate system in world coordinate system
    z_axis, ///< Eigen vector representing z axis of sensor coordinate system in world coordinate system
    sensor_position; ///< Position of the sensor in the world coordinate system
};

#endif //EG3D_SENSORINFO_H
