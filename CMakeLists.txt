cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_tutorials)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (planar_segmentation src/planar_segmentation.cpp)
target_link_libraries (planar_segmentation ${PCL_LIBRARIES})

add_executable(extract_indices src/extract_indices.cpp)
target_link_libraries (extract_indices ${PCL_LIBRARIES})

add_executable(projection src/projection.cpp)
target_link_libraries (projection ${PCL_LIBRARIES})

add_executable(concave_hull_2d src/concave_hull_2d.cpp)
target_link_libraries (concave_hull_2d ${PCL_LIBRARIES})

add_executable(extract_path src/extract_path.cpp)
target_link_libraries (extract_path ${PCL_LIBRARIES})