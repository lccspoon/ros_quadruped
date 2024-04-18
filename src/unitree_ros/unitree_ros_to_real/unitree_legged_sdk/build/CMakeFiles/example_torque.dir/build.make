# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.29

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/lcc/catkin_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lcc/catkin_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/build

# Include any dependencies generated for this target.
include CMakeFiles/example_torque.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/example_torque.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/example_torque.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/example_torque.dir/flags.make

CMakeFiles/example_torque.dir/example/example_torque.cpp.o: CMakeFiles/example_torque.dir/flags.make
CMakeFiles/example_torque.dir/example/example_torque.cpp.o: /home/lcc/catkin_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/example/example_torque.cpp
CMakeFiles/example_torque.dir/example/example_torque.cpp.o: CMakeFiles/example_torque.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/lcc/catkin_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/example_torque.dir/example/example_torque.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/example_torque.dir/example/example_torque.cpp.o -MF CMakeFiles/example_torque.dir/example/example_torque.cpp.o.d -o CMakeFiles/example_torque.dir/example/example_torque.cpp.o -c /home/lcc/catkin_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/example/example_torque.cpp

CMakeFiles/example_torque.dir/example/example_torque.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/example_torque.dir/example/example_torque.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lcc/catkin_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/example/example_torque.cpp > CMakeFiles/example_torque.dir/example/example_torque.cpp.i

CMakeFiles/example_torque.dir/example/example_torque.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/example_torque.dir/example/example_torque.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lcc/catkin_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/example/example_torque.cpp -o CMakeFiles/example_torque.dir/example/example_torque.cpp.s

# Object files for target example_torque
example_torque_OBJECTS = \
"CMakeFiles/example_torque.dir/example/example_torque.cpp.o"

# External object files for target example_torque
example_torque_EXTERNAL_OBJECTS =

devel/lib/unitree_legged_sdk/example_torque: CMakeFiles/example_torque.dir/example/example_torque.cpp.o
devel/lib/unitree_legged_sdk/example_torque: CMakeFiles/example_torque.dir/build.make
devel/lib/unitree_legged_sdk/example_torque: CMakeFiles/example_torque.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/lcc/catkin_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable devel/lib/unitree_legged_sdk/example_torque"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example_torque.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/example_torque.dir/build: devel/lib/unitree_legged_sdk/example_torque
.PHONY : CMakeFiles/example_torque.dir/build

CMakeFiles/example_torque.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/example_torque.dir/cmake_clean.cmake
.PHONY : CMakeFiles/example_torque.dir/clean

CMakeFiles/example_torque.dir/depend:
	cd /home/lcc/catkin_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lcc/catkin_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk /home/lcc/catkin_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk /home/lcc/catkin_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/build /home/lcc/catkin_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/build /home/lcc/catkin_ws/src/unitree_ros/unitree_ros_to_real/unitree_legged_sdk/build/CMakeFiles/example_torque.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/example_torque.dir/depend

