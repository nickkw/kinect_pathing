# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/nicholaskwan-wong/pcl_tutorials

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nicholaskwan-wong/pcl_tutorials/build

# Include any dependencies generated for this target.
include CMakeFiles/projection.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/projection.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/projection.dir/flags.make

CMakeFiles/projection.dir/src/projection.cpp.o: CMakeFiles/projection.dir/flags.make
CMakeFiles/projection.dir/src/projection.cpp.o: ../src/projection.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/nicholaskwan-wong/pcl_tutorials/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/projection.dir/src/projection.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/projection.dir/src/projection.cpp.o -c /home/nicholaskwan-wong/pcl_tutorials/src/projection.cpp

CMakeFiles/projection.dir/src/projection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/projection.dir/src/projection.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/nicholaskwan-wong/pcl_tutorials/src/projection.cpp > CMakeFiles/projection.dir/src/projection.cpp.i

CMakeFiles/projection.dir/src/projection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/projection.dir/src/projection.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/nicholaskwan-wong/pcl_tutorials/src/projection.cpp -o CMakeFiles/projection.dir/src/projection.cpp.s

CMakeFiles/projection.dir/src/projection.cpp.o.requires:
.PHONY : CMakeFiles/projection.dir/src/projection.cpp.o.requires

CMakeFiles/projection.dir/src/projection.cpp.o.provides: CMakeFiles/projection.dir/src/projection.cpp.o.requires
	$(MAKE) -f CMakeFiles/projection.dir/build.make CMakeFiles/projection.dir/src/projection.cpp.o.provides.build
.PHONY : CMakeFiles/projection.dir/src/projection.cpp.o.provides

CMakeFiles/projection.dir/src/projection.cpp.o.provides.build: CMakeFiles/projection.dir/src/projection.cpp.o

# Object files for target projection
projection_OBJECTS = \
"CMakeFiles/projection.dir/src/projection.cpp.o"

# External object files for target projection
projection_EXTERNAL_OBJECTS =

projection: CMakeFiles/projection.dir/src/projection.cpp.o
projection: CMakeFiles/projection.dir/build.make
projection: /usr/lib/x86_64-linux-gnu/libboost_system.so
projection: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
projection: /usr/lib/x86_64-linux-gnu/libboost_thread.so
projection: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
projection: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
projection: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
projection: /usr/lib/x86_64-linux-gnu/libpthread.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_common.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_octree.so
projection: /usr/lib/libOpenNI.so
projection: /usr/lib/libOpenNI2.so
projection: /usr/local/lib/vtk-5.10/libvtkCommon.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkFiltering.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkImaging.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkGraphics.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkGenericFiltering.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkIO.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkRendering.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkVolumeRendering.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkHybrid.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkWidgets.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkInfovis.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkGeovis.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkViews.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkCharts.so.5.10.1
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_io.so
projection: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_kdtree.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_search.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_sample_consensus.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_filters.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_features.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_ml.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_segmentation.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_visualization.so
projection: /usr/lib/x86_64-linux-gnu/libqhull.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_surface.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_registration.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_keypoints.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_tracking.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_recognition.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_stereo.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_people.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_gpu_containers.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_gpu_utils.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_gpu_octree.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_gpu_features.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_gpu_kinfu_large_scale.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_gpu_kinfu.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_gpu_segmentation.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_cuda_features.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_cuda_segmentation.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_cuda_sample_consensus.so
projection: /usr/lib/x86_64-linux-gnu/libboost_system.so
projection: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
projection: /usr/lib/x86_64-linux-gnu/libboost_thread.so
projection: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
projection: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
projection: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
projection: /usr/lib/x86_64-linux-gnu/libpthread.so
projection: /usr/lib/x86_64-linux-gnu/libqhull.so
projection: /usr/lib/libOpenNI.so
projection: /usr/lib/libOpenNI2.so
projection: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
projection: /usr/local/lib/vtk-5.10/libvtkCommon.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkFiltering.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkImaging.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkGraphics.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkGenericFiltering.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkIO.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkRendering.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkVolumeRendering.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkHybrid.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkWidgets.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkInfovis.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkGeovis.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkViews.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkCharts.so.5.10.1
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_common.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_octree.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_io.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_kdtree.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_search.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_sample_consensus.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_filters.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_features.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_ml.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_segmentation.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_visualization.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_surface.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_registration.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_keypoints.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_tracking.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_recognition.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_stereo.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_people.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_gpu_containers.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_gpu_utils.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_gpu_octree.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_gpu_features.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_gpu_kinfu_large_scale.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_gpu_kinfu.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_gpu_segmentation.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_cuda_features.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_cuda_segmentation.so
projection: /home/nicholaskwan-wong/pcl_workspace/devel_isolated/pcl/lib/libpcl_cuda_sample_consensus.so
projection: /usr/local/lib/vtk-5.10/libvtkViews.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkInfovis.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkWidgets.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkVolumeRendering.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkHybrid.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkRendering.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkImaging.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkGraphics.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkIO.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkFiltering.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtkCommon.so.5.10.1
projection: /usr/local/lib/vtk-5.10/libvtksys.so.5.10.1
projection: CMakeFiles/projection.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable projection"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/projection.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/projection.dir/build: projection
.PHONY : CMakeFiles/projection.dir/build

CMakeFiles/projection.dir/requires: CMakeFiles/projection.dir/src/projection.cpp.o.requires
.PHONY : CMakeFiles/projection.dir/requires

CMakeFiles/projection.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/projection.dir/cmake_clean.cmake
.PHONY : CMakeFiles/projection.dir/clean

CMakeFiles/projection.dir/depend:
	cd /home/nicholaskwan-wong/pcl_tutorials/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nicholaskwan-wong/pcl_tutorials /home/nicholaskwan-wong/pcl_tutorials /home/nicholaskwan-wong/pcl_tutorials/build /home/nicholaskwan-wong/pcl_tutorials/build /home/nicholaskwan-wong/pcl_tutorials/build/CMakeFiles/projection.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/projection.dir/depend

