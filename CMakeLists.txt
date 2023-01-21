cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
project(EG3D)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenMP REQUIRED)
find_package(PCL 1.12 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
file(GLOB_RECURSE SRC_FILES src/*.cpp)
set(src
    "src/BaseCloud.cpp"
    "src/RawCloud.cpp"
    "src/BoundingBox.cpp"
    "src/Region2D.cpp"
    "src/EdgeCloud.cpp")
file(GLOB_RECURSE HDR_FILES include/*.h)
set(hdrs
    "include/BaseCloud.h"
    "include/RawCloud.h"
    "include/BoundingBox.h"
    "include/Region2D.h"
    "include/EdgeCloud.h")
add_library(eg3d STATIC ${src} ${hdrs})
target_include_directories(eg3d PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(eg3d PRIVATE ${PCL_LIBRARIES} PRIVATE OpenMP::OpenMP_CXX)

add_executable(EG3D src/main.cpp)
target_include_directories(EG3D PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_BINARY_DIR})
target_link_libraries(EG3D PRIVATE eg3d PRIVATE ${PCL_LIBRARIES} PRIVATE OpenMP::OpenMP_CXX)
