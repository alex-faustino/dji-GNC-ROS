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
CMAKE_SOURCE_DIR = /home/matrice/dji_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/matrice/dji_ws/build

# Utility rule file for dji_sdk_gencpp.

# Include the progress variables for this target.
include Onboard-SDK-ROS/dji_sdk/CMakeFiles/dji_sdk_gencpp.dir/progress.make

dji_sdk_gencpp: Onboard-SDK-ROS/dji_sdk/CMakeFiles/dji_sdk_gencpp.dir/build.make

.PHONY : dji_sdk_gencpp

# Rule to build all files generated by this target.
Onboard-SDK-ROS/dji_sdk/CMakeFiles/dji_sdk_gencpp.dir/build: dji_sdk_gencpp

.PHONY : Onboard-SDK-ROS/dji_sdk/CMakeFiles/dji_sdk_gencpp.dir/build

Onboard-SDK-ROS/dji_sdk/CMakeFiles/dji_sdk_gencpp.dir/clean:
	cd /home/matrice/dji_ws/build/Onboard-SDK-ROS/dji_sdk && $(CMAKE_COMMAND) -P CMakeFiles/dji_sdk_gencpp.dir/cmake_clean.cmake
.PHONY : Onboard-SDK-ROS/dji_sdk/CMakeFiles/dji_sdk_gencpp.dir/clean

Onboard-SDK-ROS/dji_sdk/CMakeFiles/dji_sdk_gencpp.dir/depend:
	cd /home/matrice/dji_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/matrice/dji_ws/src /home/matrice/dji_ws/src/Onboard-SDK-ROS/dji_sdk /home/matrice/dji_ws/build /home/matrice/dji_ws/build/Onboard-SDK-ROS/dji_sdk /home/matrice/dji_ws/build/Onboard-SDK-ROS/dji_sdk/CMakeFiles/dji_sdk_gencpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Onboard-SDK-ROS/dji_sdk/CMakeFiles/dji_sdk_gencpp.dir/depend

