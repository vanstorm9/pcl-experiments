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
CMAKE_SOURCE_DIR = /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/spin_image/PCL_spin_image

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/spin_image/PCL_spin_image/build

# Include any dependencies generated for this target.
include CMakeFiles/computeSpinImage.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/computeSpinImage.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/computeSpinImage.dir/flags.make

CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.o: CMakeFiles/computeSpinImage.dir/flags.make
CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.o: ../src/PCL_spin_image.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/spin_image/PCL_spin_image/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.o -c /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/spin_image/PCL_spin_image/src/PCL_spin_image.cpp

CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/spin_image/PCL_spin_image/src/PCL_spin_image.cpp > CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.i

CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/spin_image/PCL_spin_image/src/PCL_spin_image.cpp -o CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.s

CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.o.requires:
.PHONY : CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.o.requires

CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.o.provides: CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.o.requires
	$(MAKE) -f CMakeFiles/computeSpinImage.dir/build.make CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.o.provides.build
.PHONY : CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.o.provides

CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.o.provides.build: CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.o

# Object files for target computeSpinImage
computeSpinImage_OBJECTS = \
"CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.o"

# External object files for target computeSpinImage
computeSpinImage_EXTERNAL_OBJECTS =

computeSpinImage: CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.o
computeSpinImage: CMakeFiles/computeSpinImage.dir/build.make
computeSpinImage: /usr/lib/x86_64-linux-gnu/libboost_system.so
computeSpinImage: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
computeSpinImage: /usr/lib/x86_64-linux-gnu/libboost_thread.so
computeSpinImage: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
computeSpinImage: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
computeSpinImage: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
computeSpinImage: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
computeSpinImage: /usr/lib/x86_64-linux-gnu/libpthread.so
computeSpinImage: /usr/local/lib/libpcl_common.so
computeSpinImage: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
computeSpinImage: /usr/local/lib/libpcl_kdtree.so
computeSpinImage: /usr/local/lib/libpcl_octree.so
computeSpinImage: /usr/local/lib/libpcl_search.so
computeSpinImage: /usr/local/lib/libpcl_sample_consensus.so
computeSpinImage: /usr/local/lib/libpcl_filters.so
computeSpinImage: /usr/lib/libOpenNI.so
computeSpinImage: /usr/lib/libOpenNI2.so
computeSpinImage: /usr/lib/libvtkCommon.so.5.8.0
computeSpinImage: /usr/lib/libvtkFiltering.so.5.8.0
computeSpinImage: /usr/lib/libvtkImaging.so.5.8.0
computeSpinImage: /usr/lib/libvtkGraphics.so.5.8.0
computeSpinImage: /usr/lib/libvtkGenericFiltering.so.5.8.0
computeSpinImage: /usr/lib/libvtkIO.so.5.8.0
computeSpinImage: /usr/lib/libvtkRendering.so.5.8.0
computeSpinImage: /usr/lib/libvtkVolumeRendering.so.5.8.0
computeSpinImage: /usr/lib/libvtkHybrid.so.5.8.0
computeSpinImage: /usr/lib/libvtkWidgets.so.5.8.0
computeSpinImage: /usr/lib/libvtkParallel.so.5.8.0
computeSpinImage: /usr/lib/libvtkInfovis.so.5.8.0
computeSpinImage: /usr/lib/libvtkGeovis.so.5.8.0
computeSpinImage: /usr/lib/libvtkViews.so.5.8.0
computeSpinImage: /usr/lib/libvtkCharts.so.5.8.0
computeSpinImage: /usr/local/lib/libpcl_io.so
computeSpinImage: /usr/local/lib/libpcl_features.so
computeSpinImage: /usr/local/lib/libpcl_visualization.so
computeSpinImage: /usr/local/lib/libpcl_ml.so
computeSpinImage: /usr/local/lib/libpcl_segmentation.so
computeSpinImage: /usr/local/lib/libpcl_people.so
computeSpinImage: /usr/local/lib/libpcl_keypoints.so
computeSpinImage: /usr/local/lib/libpcl_tracking.so
computeSpinImage: /usr/lib/x86_64-linux-gnu/libqhull.so
computeSpinImage: /usr/local/lib/libpcl_surface.so
computeSpinImage: /usr/local/lib/libpcl_outofcore.so
computeSpinImage: /usr/local/lib/libpcl_registration.so
computeSpinImage: /usr/local/lib/libpcl_recognition.so
computeSpinImage: /usr/local/lib/libpcl_stereo.so
computeSpinImage: /usr/lib/x86_64-linux-gnu/libboost_system.so
computeSpinImage: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
computeSpinImage: /usr/lib/x86_64-linux-gnu/libboost_thread.so
computeSpinImage: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
computeSpinImage: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
computeSpinImage: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
computeSpinImage: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
computeSpinImage: /usr/lib/x86_64-linux-gnu/libpthread.so
computeSpinImage: /usr/lib/x86_64-linux-gnu/libqhull.so
computeSpinImage: /usr/lib/libOpenNI.so
computeSpinImage: /usr/lib/libOpenNI2.so
computeSpinImage: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
computeSpinImage: /usr/lib/libvtkCommon.so.5.8.0
computeSpinImage: /usr/lib/libvtkFiltering.so.5.8.0
computeSpinImage: /usr/lib/libvtkImaging.so.5.8.0
computeSpinImage: /usr/lib/libvtkGraphics.so.5.8.0
computeSpinImage: /usr/lib/libvtkGenericFiltering.so.5.8.0
computeSpinImage: /usr/lib/libvtkIO.so.5.8.0
computeSpinImage: /usr/lib/libvtkRendering.so.5.8.0
computeSpinImage: /usr/lib/libvtkVolumeRendering.so.5.8.0
computeSpinImage: /usr/lib/libvtkHybrid.so.5.8.0
computeSpinImage: /usr/lib/libvtkWidgets.so.5.8.0
computeSpinImage: /usr/lib/libvtkParallel.so.5.8.0
computeSpinImage: /usr/lib/libvtkInfovis.so.5.8.0
computeSpinImage: /usr/lib/libvtkGeovis.so.5.8.0
computeSpinImage: /usr/lib/libvtkViews.so.5.8.0
computeSpinImage: /usr/lib/libvtkCharts.so.5.8.0
computeSpinImage: /usr/local/lib/libpcl_common.so
computeSpinImage: /usr/local/lib/libpcl_kdtree.so
computeSpinImage: /usr/local/lib/libpcl_octree.so
computeSpinImage: /usr/local/lib/libpcl_search.so
computeSpinImage: /usr/local/lib/libpcl_sample_consensus.so
computeSpinImage: /usr/local/lib/libpcl_filters.so
computeSpinImage: /usr/local/lib/libpcl_io.so
computeSpinImage: /usr/local/lib/libpcl_features.so
computeSpinImage: /usr/local/lib/libpcl_visualization.so
computeSpinImage: /usr/local/lib/libpcl_ml.so
computeSpinImage: /usr/local/lib/libpcl_segmentation.so
computeSpinImage: /usr/local/lib/libpcl_people.so
computeSpinImage: /usr/local/lib/libpcl_keypoints.so
computeSpinImage: /usr/local/lib/libpcl_tracking.so
computeSpinImage: /usr/local/lib/libpcl_surface.so
computeSpinImage: /usr/local/lib/libpcl_outofcore.so
computeSpinImage: /usr/local/lib/libpcl_registration.so
computeSpinImage: /usr/local/lib/libpcl_recognition.so
computeSpinImage: /usr/local/lib/libpcl_stereo.so
computeSpinImage: /usr/lib/libvtkViews.so.5.8.0
computeSpinImage: /usr/lib/libvtkInfovis.so.5.8.0
computeSpinImage: /usr/lib/libvtkWidgets.so.5.8.0
computeSpinImage: /usr/lib/libvtkVolumeRendering.so.5.8.0
computeSpinImage: /usr/lib/libvtkHybrid.so.5.8.0
computeSpinImage: /usr/lib/libvtkParallel.so.5.8.0
computeSpinImage: /usr/lib/libvtkRendering.so.5.8.0
computeSpinImage: /usr/lib/libvtkImaging.so.5.8.0
computeSpinImage: /usr/lib/libvtkGraphics.so.5.8.0
computeSpinImage: /usr/lib/libvtkIO.so.5.8.0
computeSpinImage: /usr/lib/libvtkFiltering.so.5.8.0
computeSpinImage: /usr/lib/libvtkCommon.so.5.8.0
computeSpinImage: /usr/lib/libvtksys.so.5.8.0
computeSpinImage: CMakeFiles/computeSpinImage.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable computeSpinImage"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/computeSpinImage.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/computeSpinImage.dir/build: computeSpinImage
.PHONY : CMakeFiles/computeSpinImage.dir/build

CMakeFiles/computeSpinImage.dir/requires: CMakeFiles/computeSpinImage.dir/src/PCL_spin_image.cpp.o.requires
.PHONY : CMakeFiles/computeSpinImage.dir/requires

CMakeFiles/computeSpinImage.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/computeSpinImage.dir/cmake_clean.cmake
.PHONY : CMakeFiles/computeSpinImage.dir/clean

CMakeFiles/computeSpinImage.dir/depend:
	cd /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/spin_image/PCL_spin_image/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/spin_image/PCL_spin_image /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/spin_image/PCL_spin_image /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/spin_image/PCL_spin_image/build /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/spin_image/PCL_spin_image/build /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/spin_image/PCL_spin_image/build/CMakeFiles/computeSpinImage.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/computeSpinImage.dir/depend

