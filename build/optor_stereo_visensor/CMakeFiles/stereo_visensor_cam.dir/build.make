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
CMAKE_SOURCE_DIR = /home/matrice/dji-GNC-ROS/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/matrice/dji-GNC-ROS/build

# Include any dependencies generated for this target.
include optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/depend.make

# Include the progress variables for this target.
include optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/progress.make

# Include the compile flags for this target's objects.
include optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/flags.make

optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.o: optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/flags.make
optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.o: /home/matrice/dji-GNC-ROS/src/optor_stereo_visensor/src/stereo_visensor_cam.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/matrice/dji-GNC-ROS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.o"
	cd /home/matrice/dji-GNC-ROS/build/optor_stereo_visensor && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.o -c /home/matrice/dji-GNC-ROS/src/optor_stereo_visensor/src/stereo_visensor_cam.cpp

optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.i"
	cd /home/matrice/dji-GNC-ROS/build/optor_stereo_visensor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/matrice/dji-GNC-ROS/src/optor_stereo_visensor/src/stereo_visensor_cam.cpp > CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.i

optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.s"
	cd /home/matrice/dji-GNC-ROS/build/optor_stereo_visensor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/matrice/dji-GNC-ROS/src/optor_stereo_visensor/src/stereo_visensor_cam.cpp -o CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.s

optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.o.requires:

.PHONY : optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.o.requires

optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.o.provides: optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.o.requires
	$(MAKE) -f optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/build.make optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.o.provides.build
.PHONY : optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.o.provides

optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.o.provides.build: optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.o


# Object files for target stereo_visensor_cam
stereo_visensor_cam_OBJECTS = \
"CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.o"

# External object files for target stereo_visensor_cam
stereo_visensor_cam_EXTERNAL_OBJECTS =

/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.o
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/build.make
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /home/matrice/dji-GNC-ROS/devel/lib/liboptorcam.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /home/matrice/dji-GNC-ROS/devel/lib/liboptorimu.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /home/matrice/dji-GNC-ROS/devel/lib/liboptorusb.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/libcv_bridge.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/libimage_transport.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/libclass_loader.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /usr/lib/libPocoFoundation.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /usr/lib/x86_64-linux-gnu/libdl.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/libroslib.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/librospack.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /usr/lib/x86_64-linux-gnu/libpython2.7.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/libmessage_filters.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/libroscpp.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/librosconsole.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/librostime.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/libcpp_common.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
/home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so: optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/matrice/dji-GNC-ROS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so"
	cd /home/matrice/dji-GNC-ROS/build/optor_stereo_visensor && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stereo_visensor_cam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/build: /home/matrice/dji-GNC-ROS/devel/lib/libstereo_visensor_cam.so

.PHONY : optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/build

optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/requires: optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/src/stereo_visensor_cam.cpp.o.requires

.PHONY : optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/requires

optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/clean:
	cd /home/matrice/dji-GNC-ROS/build/optor_stereo_visensor && $(CMAKE_COMMAND) -P CMakeFiles/stereo_visensor_cam.dir/cmake_clean.cmake
.PHONY : optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/clean

optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/depend:
	cd /home/matrice/dji-GNC-ROS/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/matrice/dji-GNC-ROS/src /home/matrice/dji-GNC-ROS/src/optor_stereo_visensor /home/matrice/dji-GNC-ROS/build /home/matrice/dji-GNC-ROS/build/optor_stereo_visensor /home/matrice/dji-GNC-ROS/build/optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : optor_stereo_visensor/CMakeFiles/stereo_visensor_cam.dir/depend

