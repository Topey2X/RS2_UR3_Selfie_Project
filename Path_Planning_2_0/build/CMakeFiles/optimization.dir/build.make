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
CMAKE_SOURCE_DIR = /home/darren2004/Desktop/Robotics_Studio_2/Path_Planning_2_0

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/darren2004/Desktop/Robotics_Studio_2/Path_Planning_2_0/build

# Include any dependencies generated for this target.
include CMakeFiles/optimization.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/optimization.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/optimization.dir/flags.make

CMakeFiles/optimization.dir/nearestPointbyLines.cpp.o: CMakeFiles/optimization.dir/flags.make
CMakeFiles/optimization.dir/nearestPointbyLines.cpp.o: ../nearestPointbyLines.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/darren2004/Desktop/Robotics_Studio_2/Path_Planning_2_0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/optimization.dir/nearestPointbyLines.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/optimization.dir/nearestPointbyLines.cpp.o -c /home/darren2004/Desktop/Robotics_Studio_2/Path_Planning_2_0/nearestPointbyLines.cpp

CMakeFiles/optimization.dir/nearestPointbyLines.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/optimization.dir/nearestPointbyLines.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/darren2004/Desktop/Robotics_Studio_2/Path_Planning_2_0/nearestPointbyLines.cpp > CMakeFiles/optimization.dir/nearestPointbyLines.cpp.i

CMakeFiles/optimization.dir/nearestPointbyLines.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/optimization.dir/nearestPointbyLines.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/darren2004/Desktop/Robotics_Studio_2/Path_Planning_2_0/nearestPointbyLines.cpp -o CMakeFiles/optimization.dir/nearestPointbyLines.cpp.s

# Object files for target optimization
optimization_OBJECTS = \
"CMakeFiles/optimization.dir/nearestPointbyLines.cpp.o"

# External object files for target optimization
optimization_EXTERNAL_OBJECTS =

optimization: CMakeFiles/optimization.dir/nearestPointbyLines.cpp.o
optimization: CMakeFiles/optimization.dir/build.make
optimization: CMakeFiles/optimization.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/darren2004/Desktop/Robotics_Studio_2/Path_Planning_2_0/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable optimization"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/optimization.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/optimization.dir/build: optimization

.PHONY : CMakeFiles/optimization.dir/build

CMakeFiles/optimization.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/optimization.dir/cmake_clean.cmake
.PHONY : CMakeFiles/optimization.dir/clean

CMakeFiles/optimization.dir/depend:
	cd /home/darren2004/Desktop/Robotics_Studio_2/Path_Planning_2_0/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/darren2004/Desktop/Robotics_Studio_2/Path_Planning_2_0 /home/darren2004/Desktop/Robotics_Studio_2/Path_Planning_2_0 /home/darren2004/Desktop/Robotics_Studio_2/Path_Planning_2_0/build /home/darren2004/Desktop/Robotics_Studio_2/Path_Planning_2_0/build /home/darren2004/Desktop/Robotics_Studio_2/Path_Planning_2_0/build/CMakeFiles/optimization.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/optimization.dir/depend
