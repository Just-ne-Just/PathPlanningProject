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
CMAKE_SOURCE_DIR = /home/just-ne-just/PathPlanningProject

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/just-ne-just/PathPlanningProject/Build/Release

# Include any dependencies generated for this target.
include Tests/CMakeFiles/testbench.dir/depend.make

# Include the progress variables for this target.
include Tests/CMakeFiles/testbench.dir/progress.make

# Include the compile flags for this target's objects.
include Tests/CMakeFiles/testbench.dir/flags.make

Tests/CMakeFiles/testbench.dir/testbench.cpp.o: Tests/CMakeFiles/testbench.dir/flags.make
Tests/CMakeFiles/testbench.dir/testbench.cpp.o: ../../Tests/testbench.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/just-ne-just/PathPlanningProject/Build/Release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Tests/CMakeFiles/testbench.dir/testbench.cpp.o"
	cd /home/just-ne-just/PathPlanningProject/Build/Release/Tests && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testbench.dir/testbench.cpp.o -c /home/just-ne-just/PathPlanningProject/Tests/testbench.cpp

Tests/CMakeFiles/testbench.dir/testbench.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testbench.dir/testbench.cpp.i"
	cd /home/just-ne-just/PathPlanningProject/Build/Release/Tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/just-ne-just/PathPlanningProject/Tests/testbench.cpp > CMakeFiles/testbench.dir/testbench.cpp.i

Tests/CMakeFiles/testbench.dir/testbench.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testbench.dir/testbench.cpp.s"
	cd /home/just-ne-just/PathPlanningProject/Build/Release/Tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/just-ne-just/PathPlanningProject/Tests/testbench.cpp -o CMakeFiles/testbench.dir/testbench.cpp.s

# Object files for target testbench
testbench_OBJECTS = \
"CMakeFiles/testbench.dir/testbench.cpp.o"

# External object files for target testbench
testbench_EXTERNAL_OBJECTS =

Tests/testbench: Tests/CMakeFiles/testbench.dir/testbench.cpp.o
Tests/testbench: Tests/CMakeFiles/testbench.dir/build.make
Tests/testbench: Src/libPPLIB.a
Tests/testbench: Tests/CMakeFiles/testbench.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/just-ne-just/PathPlanningProject/Build/Release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable testbench"
	cd /home/just-ne-just/PathPlanningProject/Build/Release/Tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testbench.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Tests/CMakeFiles/testbench.dir/build: Tests/testbench

.PHONY : Tests/CMakeFiles/testbench.dir/build

Tests/CMakeFiles/testbench.dir/clean:
	cd /home/just-ne-just/PathPlanningProject/Build/Release/Tests && $(CMAKE_COMMAND) -P CMakeFiles/testbench.dir/cmake_clean.cmake
.PHONY : Tests/CMakeFiles/testbench.dir/clean

Tests/CMakeFiles/testbench.dir/depend:
	cd /home/just-ne-just/PathPlanningProject/Build/Release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/just-ne-just/PathPlanningProject /home/just-ne-just/PathPlanningProject/Tests /home/just-ne-just/PathPlanningProject/Build/Release /home/just-ne-just/PathPlanningProject/Build/Release/Tests /home/just-ne-just/PathPlanningProject/Build/Release/Tests/CMakeFiles/testbench.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Tests/CMakeFiles/testbench.dir/depend

