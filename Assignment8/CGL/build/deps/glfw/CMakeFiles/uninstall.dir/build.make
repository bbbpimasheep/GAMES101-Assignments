# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.30

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
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.30.0/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.30.0/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/shizuku/Documents/Games101/Games_Homework_S2021/Homework8/Assignment8/CGL

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/shizuku/Documents/Games101/Games_Homework_S2021/Homework8/Assignment8/CGL/build

# Utility rule file for uninstall.

# Include any custom commands dependencies for this target.
include deps/glfw/CMakeFiles/uninstall.dir/compiler_depend.make

# Include the progress variables for this target.
include deps/glfw/CMakeFiles/uninstall.dir/progress.make

deps/glfw/CMakeFiles/uninstall:
	cd /Users/shizuku/Documents/Games101/Games_Homework_S2021/Homework8/Assignment8/CGL/build/deps/glfw && /opt/homebrew/Cellar/cmake/3.30.0/bin/cmake -P /Users/shizuku/Documents/Games101/Games_Homework_S2021/Homework8/Assignment8/CGL/build/deps/glfw/cmake_uninstall.cmake

uninstall: deps/glfw/CMakeFiles/uninstall
uninstall: deps/glfw/CMakeFiles/uninstall.dir/build.make
.PHONY : uninstall

# Rule to build all files generated by this target.
deps/glfw/CMakeFiles/uninstall.dir/build: uninstall
.PHONY : deps/glfw/CMakeFiles/uninstall.dir/build

deps/glfw/CMakeFiles/uninstall.dir/clean:
	cd /Users/shizuku/Documents/Games101/Games_Homework_S2021/Homework8/Assignment8/CGL/build/deps/glfw && $(CMAKE_COMMAND) -P CMakeFiles/uninstall.dir/cmake_clean.cmake
.PHONY : deps/glfw/CMakeFiles/uninstall.dir/clean

deps/glfw/CMakeFiles/uninstall.dir/depend:
	cd /Users/shizuku/Documents/Games101/Games_Homework_S2021/Homework8/Assignment8/CGL/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/shizuku/Documents/Games101/Games_Homework_S2021/Homework8/Assignment8/CGL /Users/shizuku/Documents/Games101/Games_Homework_S2021/Homework8/Assignment8/CGL/deps/glfw /Users/shizuku/Documents/Games101/Games_Homework_S2021/Homework8/Assignment8/CGL/build /Users/shizuku/Documents/Games101/Games_Homework_S2021/Homework8/Assignment8/CGL/build/deps/glfw /Users/shizuku/Documents/Games101/Games_Homework_S2021/Homework8/Assignment8/CGL/build/deps/glfw/CMakeFiles/uninstall.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : deps/glfw/CMakeFiles/uninstall.dir/depend

