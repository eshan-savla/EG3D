# EG3D

Edge, Geometry and Depth Detection in 3D point clouds <Add visualisations>

Raw unprocessed cloud:

<img src="data/rawcloud.bmp" alt="Unprocessed cloud" width="50%" height="50%" title="Raw unprocessed cloud" >

edges detected by algorithm:

<img src="data/edges.bmp" alt="edge points of cloud" width="50%" height="50%" title="Edge points">

Line segments clustered:

<img src="data/segments.bmp" alt="Line segments from edges" width="50%" height="50%" title="Line segments">

## Aim

This Project aims to extract relevant geometric and depth
information from point clouds on the basis of edge detection. Edge detection is carried out by the method proposed by
[Ni H, Lin X, et. al](https://www.mdpi.com/2072-4292/8/9/710/htm)

This library makes use of the [Point Cloud Library](https://pointclouds.org/) and parallel programming library 
[OpenMP](https://www.openmp.org/).

## Current Stand

The method in the paper was originally conceived for edge detection and line segmentation of complete point clouds.
This project attempts to adapt the method to achieve edge detection and line segmentation of point clouds which are
incomplete and are grown iteratively. Performance is improved by parallelizing edge detection and vector computation.
This project currently has the following functionality:

- Edge detection (smooth lines)
- Recognition of 2D geometric profiles in complete point clouds
- Online edge detection and line segmentation of growing point clouds
- ROS Wrapper and test environment found [here](https://gitlab.cc-asp.fraunhofer.de/RobotKit/student_projects/testeg3d).

## Future Implementations

- 3D profile information
- Classification of segmented lines into geometric profiles
- Parallelization of complete online segmentation process on the ros front