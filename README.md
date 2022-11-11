# EG3D
Edge, Geometry and Depth Detection in 3D point clouds <Add visualisations>
## Aim
This Project is a part of Eshan Savlas (chl-es) Bachelor Thesis.It aims to extract relevant geometric and depth
information from point clouds on the basis of edge detection. Edge detection is carried out by the method proposed by
[Ni H, Lin X, et. al](https://www.mdpi.com/2072-4292/8/9/710/htm)

This library makes use of the [Point Cloud Library](https://pointclouds.org/) and parallel programming library 
[OpenMP](https://www.openmp.org/).

## Current Stand
Currently, the library is capable of computing edge points of any point cloud object, albeit at a lower performance. 
Performance increase should be seen after parallelising the programme.

## Future Implementations
- Edge detection (smooth lines)
- Recognition of 2D geometric profiles
- 3D profile information
- ROS Wrapper