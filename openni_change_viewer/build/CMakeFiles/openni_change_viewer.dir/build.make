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
CMAKE_SOURCE_DIR = /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/openni_change_viewer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/openni_change_viewer/build

# Include any dependencies generated for this target.
include CMakeFiles/openni_change_viewer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/openni_change_viewer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/openni_change_viewer.dir/flags.make

CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.o: CMakeFiles/openni_change_viewer.dir/flags.make
CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.o: ../openni_change_viewer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/openni_change_viewer/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.o -c /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/openni_change_viewer/openni_change_viewer.cpp

CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/openni_change_viewer/openni_change_viewer.cpp > CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.i

CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/openni_change_viewer/openni_change_viewer.cpp -o CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.s

CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.o.requires:
.PHONY : CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.o.requires

CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.o.provides: CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.o.requires
	$(MAKE) -f CMakeFiles/openni_change_viewer.dir/build.make CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.o.provides.build
.PHONY : CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.o.provides

CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.o.provides.build: CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.o

# Object files for target openni_change_viewer
openni_change_viewer_OBJECTS = \
"CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.o"

# External object files for target openni_change_viewer
openni_change_viewer_EXTERNAL_OBJECTS =

openni_change_viewer: CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.o
openni_change_viewer: CMakeFiles/openni_change_viewer.dir/build.make
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_system.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_thread.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libpthread.so
openni_change_viewer: /usr/local/lib/libpcl_common.so
openni_change_viewer: /usr/lib/libOpenNI.so
openni_change_viewer: /usr/lib/libOpenNI2.so
openni_change_viewer: /usr/lib/libvtkCommon.so.5.8.0
openni_change_viewer: /usr/lib/libvtkFiltering.so.5.8.0
openni_change_viewer: /usr/lib/libvtkImaging.so.5.8.0
openni_change_viewer: /usr/lib/libvtkGraphics.so.5.8.0
openni_change_viewer: /usr/lib/libvtkGenericFiltering.so.5.8.0
openni_change_viewer: /usr/lib/libvtkIO.so.5.8.0
openni_change_viewer: /usr/lib/libvtkRendering.so.5.8.0
openni_change_viewer: /usr/lib/libvtkVolumeRendering.so.5.8.0
openni_change_viewer: /usr/lib/libvtkHybrid.so.5.8.0
openni_change_viewer: /usr/lib/libvtkWidgets.so.5.8.0
openni_change_viewer: /usr/lib/libvtkParallel.so.5.8.0
openni_change_viewer: /usr/lib/libvtkInfovis.so.5.8.0
openni_change_viewer: /usr/lib/libvtkGeovis.so.5.8.0
openni_change_viewer: /usr/lib/libvtkViews.so.5.8.0
openni_change_viewer: /usr/lib/libvtkCharts.so.5.8.0
openni_change_viewer: /usr/local/lib/libpcl_io.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_system.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_thread.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libpthread.so
openni_change_viewer: /usr/local/lib/libpcl_common.so
openni_change_viewer: /usr/local/lib/libpcl_octree.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_system.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_thread.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
openni_change_viewer: /usr/lib/x86_64-linux-gnu/libpthread.so
openni_change_viewer: /usr/local/lib/libpcl_common.so
openni_change_viewer: /usr/lib/libOpenNI.so
openni_change_viewer: /usr/lib/libOpenNI2.so
openni_change_viewer: /usr/local/lib/libpcl_io.so
openni_change_viewer: /usr/local/lib/libpcl_octree.so
openni_change_viewer: /usr/lib/libvtkViews.so.5.8.0
openni_change_viewer: /usr/lib/libvtkInfovis.so.5.8.0
openni_change_viewer: /usr/lib/libvtkWidgets.so.5.8.0
openni_change_viewer: /usr/lib/libvtkVolumeRendering.so.5.8.0
openni_change_viewer: /usr/lib/libvtkHybrid.so.5.8.0
openni_change_viewer: /usr/lib/libvtkParallel.so.5.8.0
openni_change_viewer: /usr/lib/libvtkRendering.so.5.8.0
openni_change_viewer: /usr/lib/libvtkImaging.so.5.8.0
openni_change_viewer: /usr/lib/libvtkGraphics.so.5.8.0
openni_change_viewer: /usr/lib/libvtkIO.so.5.8.0
openni_change_viewer: /usr/lib/libvtkFiltering.so.5.8.0
openni_change_viewer: /usr/lib/libvtkCommon.so.5.8.0
openni_change_viewer: /usr/lib/libvtksys.so.5.8.0
openni_change_viewer: CMakeFiles/openni_change_viewer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable openni_change_viewer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/openni_change_viewer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/openni_change_viewer.dir/build: openni_change_viewer
.PHONY : CMakeFiles/openni_change_viewer.dir/build

CMakeFiles/openni_change_viewer.dir/requires: CMakeFiles/openni_change_viewer.dir/openni_change_viewer.cpp.o.requires
.PHONY : CMakeFiles/openni_change_viewer.dir/requires

CMakeFiles/openni_change_viewer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/openni_change_viewer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/openni_change_viewer.dir/clean

CMakeFiles/openni_change_viewer.dir/depend:
	cd /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/openni_change_viewer/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/openni_change_viewer /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/openni_change_viewer /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/openni_change_viewer/build /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/openni_change_viewer/build /home/anthony/Documents/Programming/Kinect/C/test-scripts/pcl-experiments/openni_change_viewer/build/CMakeFiles/openni_change_viewer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/openni_change_viewer.dir/depend

