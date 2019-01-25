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

# Include any dependencies generated for this target.
include optor_stereo_visensor/CMakeFiles/optorimu.dir/depend.make

# Include the progress variables for this target.
include optor_stereo_visensor/CMakeFiles/optorimu.dir/progress.make

# Include the compile flags for this target's objects.
include optor_stereo_visensor/CMakeFiles/optorimu.dir/flags.make

optor_stereo_visensor/CMakeFiles/optorimu.dir/src/optorimu.cpp.o: optor_stereo_visensor/CMakeFiles/optorimu.dir/flags.make
optor_stereo_visensor/CMakeFiles/optorimu.dir/src/optorimu.cpp.o: /home/matrice/dji_ws/src/optor_stereo_visensor/src/optorimu.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/matrice/dji_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object optor_stereo_visensor/CMakeFiles/optorimu.dir/src/optorimu.cpp.o"
	cd /home/matrice/dji_ws/build/optor_stereo_visensor && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/optorimu.dir/src/optorimu.cpp.o -c /home/matrice/dji_ws/src/optor_stereo_visensor/src/optorimu.cpp

optor_stereo_visensor/CMakeFiles/optorimu.dir/src/optorimu.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/optorimu.dir/src/optorimu.cpp.i"
	cd /home/matrice/dji_ws/build/optor_stereo_visensor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/matrice/dji_ws/src/optor_stereo_visensor/src/optorimu.cpp > CMakeFiles/optorimu.dir/src/optorimu.cpp.i

optor_stereo_visensor/CMakeFiles/optorimu.dir/src/optorimu.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/optorimu.dir/src/optorimu.cpp.s"
	cd /home/matrice/dji_ws/build/optor_stereo_visensor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/matrice/dji_ws/src/optor_stereo_visensor/src/optorimu.cpp -o CMakeFiles/optorimu.dir/src/optorimu.cpp.s

optor_stereo_visensor/CMakeFiles/optorimu.dir/src/optorimu.cpp.o.requires:

.PHONY : optor_stereo_visensor/CMakeFiles/optorimu.dir/src/optorimu.cpp.o.requires

optor_stereo_visensor/CMakeFiles/optorimu.dir/src/optorimu.cpp.o.provides: optor_stereo_visensor/CMakeFiles/optorimu.dir/src/optorimu.cpp.o.requires
	$(MAKE) -f optor_stereo_visensor/CMakeFiles/optorimu.dir/build.make optor_stereo_visensor/CMakeFiles/optorimu.dir/src/optorimu.cpp.o.provides.build
.PHONY : optor_stereo_visensor/CMakeFiles/optorimu.dir/src/optorimu.cpp.o.provides

optor_stereo_visensor/CMakeFiles/optorimu.dir/src/optorimu.cpp.o.provides.build: optor_stereo_visensor/CMakeFiles/optorimu.dir/src/optorimu.cpp.o


# Object files for target optorimu
optorimu_OBJECTS = \
"CMakeFiles/optorimu.dir/src/optorimu.cpp.o"

# External object files for target optorimu
optorimu_EXTERNAL_OBJECTS =

/home/matrice/dji_ws/devel/lib/liboptorimu.so: optor_stereo_visensor/CMakeFiles/optorimu.dir/src/optorimu.cpp.o
/home/matrice/dji_ws/devel/lib/liboptorimu.so: optor_stereo_visensor/CMakeFiles/optorimu.dir/build.make
/home/matrice/dji_ws/devel/lib/liboptorimu.so: optor_stereo_visensor/CMakeFiles/optorimu.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/matrice/dji_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/matrice/dji_ws/devel/lib/liboptorimu.so"
	cd /home/matrice/dji_ws/build/optor_stereo_visensor && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/optorimu.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
optor_stereo_visensor/CMakeFiles/optorimu.dir/build: /home/matrice/dji_ws/devel/lib/liboptorimu.so

.PHONY : optor_stereo_visensor/CMakeFiles/optorimu.dir/build

optor_stereo_visensor/CMakeFiles/optorimu.dir/requires: optor_stereo_visensor/CMakeFiles/optorimu.dir/src/optorimu.cpp.o.requires

.PHONY : optor_stereo_visensor/CMakeFiles/optorimu.dir/requires

optor_stereo_visensor/CMakeFiles/optorimu.dir/clean:
	cd /home/matrice/dji_ws/build/optor_stereo_visensor && $(CMAKE_COMMAND) -P CMakeFiles/optorimu.dir/cmake_clean.cmake
.PHONY : optor_stereo_visensor/CMakeFiles/optorimu.dir/clean

optor_stereo_visensor/CMakeFiles/optorimu.dir/depend:
	cd /home/matrice/dji_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/matrice/dji_ws/src /home/matrice/dji_ws/src/optor_stereo_visensor /home/matrice/dji_ws/build /home/matrice/dji_ws/build/optor_stereo_visensor /home/matrice/dji_ws/build/optor_stereo_visensor/CMakeFiles/optorimu.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : optor_stereo_visensor/CMakeFiles/optorimu.dir/depend
