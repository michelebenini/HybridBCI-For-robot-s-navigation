# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/michele/catkin_ws/src/hybrid_bci

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/michele/catkin_ws/src/hybrid_bci/build

# Include any dependencies generated for this target.
include CMakeFiles/cmd_simu.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cmd_simu.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cmd_simu.dir/flags.make

CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.o: CMakeFiles/cmd_simu.dir/flags.make
CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.o: ../src/cmd_simu.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/michele/catkin_ws/src/hybrid_bci/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.o -c /home/michele/catkin_ws/src/hybrid_bci/src/cmd_simu.cpp

CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/michele/catkin_ws/src/hybrid_bci/src/cmd_simu.cpp > CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.i

CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/michele/catkin_ws/src/hybrid_bci/src/cmd_simu.cpp -o CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.s

CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.o.requires:

.PHONY : CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.o.requires

CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.o.provides: CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.o.requires
	$(MAKE) -f CMakeFiles/cmd_simu.dir/build.make CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.o.provides.build
.PHONY : CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.o.provides

CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.o.provides.build: CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.o


# Object files for target cmd_simu
cmd_simu_OBJECTS = \
"CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.o"

# External object files for target cmd_simu
cmd_simu_EXTERNAL_OBJECTS =

devel/lib/hybrid_bci/cmd_simu: CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.o
devel/lib/hybrid_bci/cmd_simu: CMakeFiles/cmd_simu.dir/build.make
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.9
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.9
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.9
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.9
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.9
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.9
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/libdynamic_reconfigure_config_init_mutex.so
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/libcv_bridge.so
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/libimage_transport.so
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/libclass_loader.so
devel/lib/hybrid_bci/cmd_simu: /usr/lib/libPocoFoundation.so
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libdl.so
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/libroslib.so
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/librospack.so
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libpython2.7.so
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libtinyxml.so
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/libtf.so
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/libtf2_ros.so
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/libactionlib.so
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/libmessage_filters.so
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/libroscpp.so
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libboost_signals.so
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/libxmlrpcpp.so
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/libtf2.so
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/librosconsole.so
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/libroscpp_serialization.so
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/librostime.so
devel/lib/hybrid_bci/cmd_simu: /opt/ros/kinetic/lib/libcpp_common.so
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.9
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.9
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.9
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.9
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.9
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.9
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.9
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.9
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.9
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.9
devel/lib/hybrid_bci/cmd_simu: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.9
devel/lib/hybrid_bci/cmd_simu: CMakeFiles/cmd_simu.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/michele/catkin_ws/src/hybrid_bci/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable devel/lib/hybrid_bci/cmd_simu"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cmd_simu.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cmd_simu.dir/build: devel/lib/hybrid_bci/cmd_simu

.PHONY : CMakeFiles/cmd_simu.dir/build

CMakeFiles/cmd_simu.dir/requires: CMakeFiles/cmd_simu.dir/src/cmd_simu.cpp.o.requires

.PHONY : CMakeFiles/cmd_simu.dir/requires

CMakeFiles/cmd_simu.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cmd_simu.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cmd_simu.dir/clean

CMakeFiles/cmd_simu.dir/depend:
	cd /home/michele/catkin_ws/src/hybrid_bci/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/michele/catkin_ws/src/hybrid_bci /home/michele/catkin_ws/src/hybrid_bci /home/michele/catkin_ws/src/hybrid_bci/build /home/michele/catkin_ws/src/hybrid_bci/build /home/michele/catkin_ws/src/hybrid_bci/build/CMakeFiles/cmd_simu.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cmd_simu.dir/depend

