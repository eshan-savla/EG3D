cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(EG3D)

set(CMAKE_CXX_STANDARD 14)

find_package(PCL 1.7 REQUIRED COMPONENTS segmentation sample_consensus kdtree visualization filters common io)
include_directories("${PCL_INCLUDE_DIRS}" "${PROJECT_SOURCE_DIR}/include")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
file(GLOB_RECURSE SRC_FILES src/*.cpp)
file(GLOB_RECURSE HDR_FILES include/*.h)
add_executable(EG3D src/main.cpp ${SRC_FILES} ${HDR_FILES})
target_include_directories(EG3D PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(EG3D ${PCL_LIBRARIES})
