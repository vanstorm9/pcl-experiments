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
CMAKE_SOURCE_DIR = /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/passthrough_filter

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/passthrough_filter/build

# Include any dependencies generated for this target.
include CMakeFiles/passthroughFilter.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/passthroughFilter.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/passthroughFilter.dir/flags.make

CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.o: CMakeFiles/passthroughFilter.dir/flags.make
CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.o: ../src/PCL_passthrough_filter.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/passthrough_filter/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.o -c /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/passthrough_filter/src/PCL_passthrough_filter.cpp

CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/passthrough_filter/src/PCL_passthrough_filter.cpp > CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.i

CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/passthrough_filter/src/PCL_passthrough_filter.cpp -o CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.s

CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.o.requires:
.PHONY : CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.o.requires

CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.o.provides: CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.o.requires
	$(MAKE) -f CMakeFiles/passthroughFilter.dir/build.make CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.o.provides.build
.PHONY : CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.o.provides

CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.o.provides.build: CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.o

# Object files for target passthroughFilter
passthroughFilter_OBJECTS = \
"CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.o"

# External object files for target passthroughFilter
passthroughFilter_EXTERNAL_OBJECTS =

passthroughFilter: CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.o
passthroughFilter: CMakeFiles/passthroughFilter.dir/build.make
passthroughFilter: /usr/lib/x86_64-linux-gnu/libboost_system.so
passthroughFilter: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
passthroughFilter: /usr/lib/x86_64-linux-gnu/libboost_thread.so
passthroughFilter: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
passthroughFilter: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
passthroughFilter: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
passthroughFilter: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
passthroughFilter: /usr/lib/x86_64-linux-gnu/libpthread.so
passthroughFilter: /usr/local/lib/libpcl_common.so
passthroughFilter: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
passthroughFilter: /usr/local/lib/libpcl_kdtree.so
passthroughFilter: /usr/local/lib/libpcl_octree.so
passthroughFilter: /usr/local/lib/libpcl_search.so
passthroughFilter: /usr/local/lib/libpcl_sample_consensus.so
passthroughFilter: /usr/local/lib/libpcl_filters.so
passthroughFilter: /usr/lib/libOpenNI.so
passthroughFilter: /usr/lib/libOpenNI2.so
passthroughFilter: /usr/lib/libvtkCommon.so.5.8.0
passthroughFilter: /usr/lib/libvtkFiltering.so.5.8.0
passthroughFilter: /usr/lib/libvtkImaging.so.5.8.0
passthroughFilter: /usr/lib/libvtkGraphics.so.5.8.0
passthroughFilter: /usr/lib/libvtkGenericFiltering.so.5.8.0
passthroughFilter: /usr/lib/libvtkIO.so.5.8.0
passthroughFilter: /usr/lib/libvtkRendering.so.5.8.0
passthroughFilter: /usr/lib/libvtkVolumeRendering.so.5.8.0
passthroughFilter: /usr/lib/libvtkHybrid.so.5.8.0
passthroughFilter: /usr/lib/libvtkWidgets.so.5.8.0
passthroughFilter: /usr/lib/libvtkParallel.so.5.8.0
passthroughFilter: /usr/lib/libvtkInfovis.so.5.8.0
passthroughFilter: /usr/lib/libvtkGeovis.so.5.8.0
passthroughFilter: /usr/lib/libvtkViews.so.5.8.0
passthroughFilter: /usr/lib/libvtkCharts.so.5.8.0
passthroughFilter: /usr/local/lib/libpcl_io.so
passthroughFilter: /usr/local/lib/libpcl_features.so
passthroughFilter: /usr/local/lib/libpcl_visualization.so
passthroughFilter: /usr/local/lib/libpcl_ml.so
passthroughFilter: /usr/local/lib/libpcl_segmentation.so
passthroughFilter: /usr/local/lib/libpcl_people.so
passthroughFilter: /usr/local/lib/libpcl_keypoints.so
passthroughFilter: /usr/local/lib/libpcl_tracking.so
passthroughFilter: /usr/lib/x86_64-linux-gnu/libqhull.so
passthroughFilter: /usr/local/lib/libpcl_surface.so
passthroughFilter: /usr/local/lib/libpcl_outofcore.so
passthroughFilter: /usr/local/lib/libpcl_registration.so
passthroughFilter: /usr/local/lib/libpcl_recognition.so
passthroughFilter: /usr/local/lib/libpcl_stereo.so
passthroughFilter: /usr/lib/x86_64-linux-gnu/libboost_system.so
passthroughFilter: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
passthroughFilter: /usr/lib/x86_64-linux-gnu/libboost_thread.so
passthroughFilter: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
passthroughFilter: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
passthroughFilter: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
passthroughFilter: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
passthroughFilter: /usr/lib/x86_64-linux-gnu/libpthread.so
passthroughFilter: /usr/lib/x86_64-linux-gnu/libqhull.so
passthroughFilter: /usr/lib/libOpenNI.so
passthroughFilter: /usr/lib/libOpenNI2.so
passthroughFilter: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
passthroughFilter: /usr/lib/libvtkCommon.so.5.8.0
passthroughFilter: /usr/lib/libvtkFiltering.so.5.8.0
passthroughFilter: /usr/lib/libvtkImaging.so.5.8.0
passthroughFilter: /usr/lib/libvtkGraphics.so.5.8.0
passthroughFilter: /usr/lib/libvtkGenericFiltering.so.5.8.0
passthroughFilter: /usr/lib/libvtkIO.so.5.8.0
passthroughFilter: /usr/lib/libvtkRendering.so.5.8.0
passthroughFilter: /usr/lib/libvtkVolumeRendering.so.5.8.0
passthroughFilter: /usr/lib/libvtkHybrid.so.5.8.0
passthroughFilter: /usr/lib/libvtkWidgets.so.5.8.0
passthroughFilter: /usr/lib/libvtkParallel.so.5.8.0
passthroughFilter: /usr/lib/libvtkInfovis.so.5.8.0
passthroughFilter: /usr/lib/libvtkGeovis.so.5.8.0
passthroughFilter: /usr/lib/libvtkViews.so.5.8.0
passthroughFilter: /usr/lib/libvtkCharts.so.5.8.0
passthroughFilter: /usr/local/lib/libpcl_common.so
passthroughFilter: /usr/local/lib/libpcl_kdtree.so
passthroughFilter: /usr/local/lib/libpcl_octree.so
passthroughFilter: /usr/local/lib/libpcl_search.so
passthroughFilter: /usr/local/lib/libpcl_sample_consensus.so
passthroughFilter: /usr/local/lib/libpcl_filters.so
passthroughFilter: /usr/local/lib/libpcl_io.so
passthroughFilter: /usr/local/lib/libpcl_features.so
passthroughFilter: /usr/local/lib/libpcl_visualization.so
passthroughFilter: /usr/local/lib/libpcl_ml.so
passthroughFilter: /usr/local/lib/libpcl_segmentation.so
passthroughFilter: /usr/local/lib/libpcl_people.so
passthroughFilter: /usr/local/lib/libpcl_keypoints.so
passthroughFilter: /usr/local/lib/libpcl_tracking.so
passthroughFilter: /usr/local/lib/libpcl_surface.so
passthroughFilter: /usr/local/lib/libpcl_outofcore.so
passthroughFilter: /usr/local/lib/libpcl_registration.so
passthroughFilter: /usr/local/lib/libpcl_recognition.so
passthroughFilter: /usr/local/lib/libpcl_stereo.so
passthroughFilter: /usr/lib/libvtkViews.so.5.8.0
passthroughFilter: /usr/lib/libvtkInfovis.so.5.8.0
passthroughFilter: /usr/lib/libvtkWidgets.so.5.8.0
passthroughFilter: /usr/lib/libvtkVolumeRendering.so.5.8.0
passthroughFilter: /usr/lib/libvtkHybrid.so.5.8.0
passthroughFilter: /usr/lib/libvtkParallel.so.5.8.0
passthroughFilter: /usr/lib/libvtkRendering.so.5.8.0
passthroughFilter: /usr/lib/libvtkImaging.so.5.8.0
passthroughFilter: /usr/lib/libvtkGraphics.so.5.8.0
passthroughFilter: /usr/lib/libvtkIO.so.5.8.0
passthroughFilter: /usr/lib/libvtkFiltering.so.5.8.0
passthroughFilter: /usr/lib/libvtkCommon.so.5.8.0
passthroughFilter: /usr/lib/libvtksys.so.5.8.0
passthroughFilter: CMakeFiles/passthroughFilter.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable passthroughFilter"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/passthroughFilter.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/passthroughFilter.dir/build: passthroughFilter
.PHONY : CMakeFiles/passthroughFilter.dir/build

CMakeFiles/passthroughFilter.dir/requires: CMakeFiles/passthroughFilter.dir/src/PCL_passthrough_filter.cpp.o.requires
.PHONY : CMakeFiles/passthroughFilter.dir/requires

CMakeFiles/passthroughFilter.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/passthroughFilter.dir/cmake_clean.cmake
.PHONY : CMakeFiles/passthroughFilter.dir/clean

CMakeFiles/passthroughFilter.dir/depend:
	cd /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/passthrough_filter/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/passthrough_filter /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/passthrough_filter /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/passthrough_filter/build /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/passthrough_filter/build /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/passthrough_filter/build/CMakeFiles/passthroughFilter.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/passthroughFilter.dir/depend

