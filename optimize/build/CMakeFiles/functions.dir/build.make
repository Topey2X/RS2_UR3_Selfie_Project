# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/darren2004/catkin_ws/src/optimize

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/darren2004/catkin_ws/src/optimize/build

# Include any dependencies generated for this target.
include CMakeFiles/functions.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/functions.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/functions.dir/flags.make

CMakeFiles/functions.dir/src/function.cpp.o: CMakeFiles/functions.dir/flags.make
CMakeFiles/functions.dir/src/function.cpp.o: ../src/function.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/darren2004/catkin_ws/src/optimize/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/functions.dir/src/function.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/functions.dir/src/function.cpp.o -c /home/darren2004/catkin_ws/src/optimize/src/function.cpp

CMakeFiles/functions.dir/src/function.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/functions.dir/src/function.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/darren2004/catkin_ws/src/optimize/src/function.cpp > CMakeFiles/functions.dir/src/function.cpp.i

CMakeFiles/functions.dir/src/function.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/functions.dir/src/function.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/darren2004/catkin_ws/src/optimize/src/function.cpp -o CMakeFiles/functions.dir/src/function.cpp.s

# Object files for target functions
functions_OBJECTS = \
"CMakeFiles/functions.dir/src/function.cpp.o"

# External object files for target functions
functions_EXTERNAL_OBJECTS =

devel/lib/libfunctions.so: CMakeFiles/functions.dir/src/function.cpp.o
devel/lib/libfunctions.so: CMakeFiles/functions.dir/build.make
devel/lib/libfunctions.so: CMakeFiles/functions.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/darren2004/catkin_ws/src/optimize/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library devel/lib/libfunctions.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/functions.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/functions.dir/build: devel/lib/libfunctions.so

.PHONY : CMakeFiles/functions.dir/build

CMakeFiles/functions.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/functions.dir/cmake_clean.cmake
.PHONY : CMakeFiles/functions.dir/clean

CMakeFiles/functions.dir/depend:
	cd /home/darren2004/catkin_ws/src/optimize/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/darren2004/catkin_ws/src/optimize /home/darren2004/catkin_ws/src/optimize /home/darren2004/catkin_ws/src/optimize/build /home/darren2004/catkin_ws/src/optimize/build /home/darren2004/catkin_ws/src/optimize/build/CMakeFiles/functions.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/functions.dir/depend

