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
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/octree_spatial_change

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/octree_spatial_change/build

# Include any dependencies generated for this target.
include CMakeFiles/octree_change_detection.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/octree_change_detection.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/octree_change_detection.dir/flags.make

CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.o: CMakeFiles/octree_change_detection.dir/flags.make
CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.o: ../octree_change_detection.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/octree_spatial_change/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.o -c /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/octree_spatial_change/octree_change_detection.cpp

CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/octree_spatial_change/octree_change_detection.cpp > CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.i

CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/octree_spatial_change/octree_change_detection.cpp -o CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.s

CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.o.requires:
.PHONY : CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.o.requires

CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.o.provides: CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.o.requires
	$(MAKE) -f CMakeFiles/octree_change_detection.dir/build.make CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.o.provides.build
.PHONY : CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.o.provides

CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.o.provides.build: CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.o

# Object files for target octree_change_detection
octree_change_detection_OBJECTS = \
"CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.o"

# External object files for target octree_change_detection
octree_change_detection_EXTERNAL_OBJECTS =

octree_change_detection: CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.o
octree_change_detection: CMakeFiles/octree_change_detection.dir/build.make
octree_change_detection: /usr/lib/x86_64-linux-gnu/libboost_system.so
octree_change_detection: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
octree_change_detection: /usr/lib/x86_64-linux-gnu/libboost_thread.so
octree_change_detection: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
octree_change_detection: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
octree_change_detection: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
octree_change_detection: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
octree_change_detection: /usr/lib/x86_64-linux-gnu/libpthread.so
octree_change_detection: /usr/local/lib/libpcl_common.so
octree_change_detection: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
octree_change_detection: /usr/local/lib/libpcl_kdtree.so
octree_change_detection: /usr/local/lib/libpcl_octree.so
octree_change_detection: /usr/local/lib/libpcl_search.so
octree_change_detection: /usr/local/lib/libpcl_sample_consensus.so
octree_change_detection: /usr/local/lib/libpcl_filters.so
octree_change_detection: /usr/lib/libOpenNI.so
octree_change_detection: /usr/lib/libOpenNI2.so
octree_change_detection: /usr/lib/libvtkCommon.so.5.8.0
octree_change_detection: /usr/lib/libvtkFiltering.so.5.8.0
octree_change_detection: /usr/lib/libvtkImaging.so.5.8.0
octree_change_detection: /usr/lib/libvtkGraphics.so.5.8.0
octree_change_detection: /usr/lib/libvtkGenericFiltering.so.5.8.0
octree_change_detection: /usr/lib/libvtkIO.so.5.8.0
octree_change_detection: /usr/lib/libvtkRendering.so.5.8.0
octree_change_detection: /usr/lib/libvtkVolumeRendering.so.5.8.0
octree_change_detection: /usr/lib/libvtkHybrid.so.5.8.0
octree_change_detection: /usr/lib/libvtkWidgets.so.5.8.0
octree_change_detection: /usr/lib/libvtkParallel.so.5.8.0
octree_change_detection: /usr/lib/libvtkInfovis.so.5.8.0
octree_change_detection: /usr/lib/libvtkGeovis.so.5.8.0
octree_change_detection: /usr/lib/libvtkViews.so.5.8.0
octree_change_detection: /usr/lib/libvtkCharts.so.5.8.0
octree_change_detection: /usr/local/lib/libpcl_io.so
octree_change_detection: /usr/local/lib/libpcl_features.so
octree_change_detection: /usr/local/lib/libpcl_visualization.so
octree_change_detection: /usr/local/lib/libpcl_ml.so
octree_change_detection: /usr/local/lib/libpcl_segmentation.so
octree_change_detection: /usr/local/lib/libpcl_people.so
octree_change_detection: /usr/local/lib/libpcl_keypoints.so
octree_change_detection: /usr/local/lib/libpcl_tracking.so
octree_change_detection: /usr/lib/x86_64-linux-gnu/libqhull.so
octree_change_detection: /usr/local/lib/libpcl_surface.so
octree_change_detection: /usr/local/lib/libpcl_outofcore.so
octree_change_detection: /usr/local/lib/libpcl_registration.so
octree_change_detection: /usr/local/lib/libpcl_recognition.so
octree_change_detection: /usr/local/lib/libpcl_stereo.so
octree_change_detection: /usr/lib/x86_64-linux-gnu/libboost_system.so
octree_change_detection: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
octree_change_detection: /usr/lib/x86_64-linux-gnu/libboost_thread.so
octree_change_detection: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
octree_change_detection: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
octree_change_detection: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
octree_change_detection: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
octree_change_detection: /usr/lib/x86_64-linux-gnu/libpthread.so
octree_change_detection: /usr/lib/x86_64-linux-gnu/libqhull.so
octree_change_detection: /usr/lib/libOpenNI.so
octree_change_detection: /usr/lib/libOpenNI2.so
octree_change_detection: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
octree_change_detection: /usr/lib/libvtkCommon.so.5.8.0
octree_change_detection: /usr/lib/libvtkFiltering.so.5.8.0
octree_change_detection: /usr/lib/libvtkImaging.so.5.8.0
octree_change_detection: /usr/lib/libvtkGraphics.so.5.8.0
octree_change_detection: /usr/lib/libvtkGenericFiltering.so.5.8.0
octree_change_detection: /usr/lib/libvtkIO.so.5.8.0
octree_change_detection: /usr/lib/libvtkRendering.so.5.8.0
octree_change_detection: /usr/lib/libvtkVolumeRendering.so.5.8.0
octree_change_detection: /usr/lib/libvtkHybrid.so.5.8.0
octree_change_detection: /usr/lib/libvtkWidgets.so.5.8.0
octree_change_detection: /usr/lib/libvtkParallel.so.5.8.0
octree_change_detection: /usr/lib/libvtkInfovis.so.5.8.0
octree_change_detection: /usr/lib/libvtkGeovis.so.5.8.0
octree_change_detection: /usr/lib/libvtkViews.so.5.8.0
octree_change_detection: /usr/lib/libvtkCharts.so.5.8.0
octree_change_detection: /usr/local/lib/libpcl_common.so
octree_change_detection: /usr/local/lib/libpcl_kdtree.so
octree_change_detection: /usr/local/lib/libpcl_octree.so
octree_change_detection: /usr/local/lib/libpcl_search.so
octree_change_detection: /usr/local/lib/libpcl_sample_consensus.so
octree_change_detection: /usr/local/lib/libpcl_filters.so
octree_change_detection: /usr/local/lib/libpcl_io.so
octree_change_detection: /usr/local/lib/libpcl_features.so
octree_change_detection: /usr/local/lib/libpcl_visualization.so
octree_change_detection: /usr/local/lib/libpcl_ml.so
octree_change_detection: /usr/local/lib/libpcl_segmentation.so
octree_change_detection: /usr/local/lib/libpcl_people.so
octree_change_detection: /usr/local/lib/libpcl_keypoints.so
octree_change_detection: /usr/local/lib/libpcl_tracking.so
octree_change_detection: /usr/local/lib/libpcl_surface.so
octree_change_detection: /usr/local/lib/libpcl_outofcore.so
octree_change_detection: /usr/local/lib/libpcl_registration.so
octree_change_detection: /usr/local/lib/libpcl_recognition.so
octree_change_detection: /usr/local/lib/libpcl_stereo.so
octree_change_detection: /usr/lib/libvtkViews.so.5.8.0
octree_change_detection: /usr/lib/libvtkInfovis.so.5.8.0
octree_change_detection: /usr/lib/libvtkWidgets.so.5.8.0
octree_change_detection: /usr/lib/libvtkVolumeRendering.so.5.8.0
octree_change_detection: /usr/lib/libvtkHybrid.so.5.8.0
octree_change_detection: /usr/lib/libvtkParallel.so.5.8.0
octree_change_detection: /usr/lib/libvtkRendering.so.5.8.0
octree_change_detection: /usr/lib/libvtkImaging.so.5.8.0
octree_change_detection: /usr/lib/libvtkGraphics.so.5.8.0
octree_change_detection: /usr/lib/libvtkIO.so.5.8.0
octree_change_detection: /usr/lib/libvtkFiltering.so.5.8.0
octree_change_detection: /usr/lib/libvtkCommon.so.5.8.0
octree_change_detection: /usr/lib/libvtksys.so.5.8.0
octree_change_detection: CMakeFiles/octree_change_detection.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable octree_change_detection"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/octree_change_detection.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/octree_change_detection.dir/build: octree_change_detection
.PHONY : CMakeFiles/octree_change_detection.dir/build

CMakeFiles/octree_change_detection.dir/requires: CMakeFiles/octree_change_detection.dir/octree_change_detection.cpp.o.requires
.PHONY : CMakeFiles/octree_change_detection.dir/requires

CMakeFiles/octree_change_detection.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/octree_change_detection.dir/cmake_clean.cmake
.PHONY : CMakeFiles/octree_change_detection.dir/clean

CMakeFiles/octree_change_detection.dir/depend:
	cd /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/octree_spatial_change/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/octree_spatial_change /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/octree_spatial_change /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/octree_spatial_change/build /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/octree_spatial_change/build /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/octree_spatial_change/build/CMakeFiles/octree_change_detection.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/octree_change_detection.dir/depend

