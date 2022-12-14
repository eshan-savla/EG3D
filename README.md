# EG3D
Edge, Geometry and Depth Detection in 3D point clouds <Add visualisations>
## Aim
This Project is a part of Eshan Savlas (chl-es) Bachelor Thesis.It aims to extract relevant geometric and depth
information from point clouds on the basis of edge detection. Edge detection is carried out by the method proposed by
[Ni H, Lin X, et. al](https://www.mdpi.com/2072-4292/8/9/710/htm)

This library makes use of the [Point Cloud Library](https://pointclouds.org/) and parallel programming library 
[OpenMP](https://www.openmp.org/).

## Current Stand
Currently, the library is capable of isolating edge points and segmenting feature lines from it. 2D as well as 3D
Information can be derived from the segments but is not within the scope of this thesis.

## Future Implementations
- online edge detection
- ROS Wrapper