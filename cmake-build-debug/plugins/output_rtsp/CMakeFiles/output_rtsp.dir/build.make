# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_COMMAND = /home/feilong/下载/clion-2017.1.1/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/feilong/下载/clion-2017.1.1/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/feilong/文档/workspace/fr/mjpg-streamer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/feilong/文档/workspace/fr/mjpg-streamer/cmake-build-debug

# Include any dependencies generated for this target.
include plugins/output_rtsp/CMakeFiles/output_rtsp.dir/depend.make

# Include the progress variables for this target.
include plugins/output_rtsp/CMakeFiles/output_rtsp.dir/progress.make

# Include the compile flags for this target's objects.
include plugins/output_rtsp/CMakeFiles/output_rtsp.dir/flags.make

plugins/output_rtsp/CMakeFiles/output_rtsp.dir/output_rtsp.c.o: plugins/output_rtsp/CMakeFiles/output_rtsp.dir/flags.make
plugins/output_rtsp/CMakeFiles/output_rtsp.dir/output_rtsp.c.o: ../plugins/output_rtsp/output_rtsp.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/feilong/文档/workspace/fr/mjpg-streamer/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object plugins/output_rtsp/CMakeFiles/output_rtsp.dir/output_rtsp.c.o"
	cd /home/feilong/文档/workspace/fr/mjpg-streamer/cmake-build-debug/plugins/output_rtsp && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/output_rtsp.dir/output_rtsp.c.o   -c /home/feilong/文档/workspace/fr/mjpg-streamer/plugins/output_rtsp/output_rtsp.c

plugins/output_rtsp/CMakeFiles/output_rtsp.dir/output_rtsp.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/output_rtsp.dir/output_rtsp.c.i"
	cd /home/feilong/文档/workspace/fr/mjpg-streamer/cmake-build-debug/plugins/output_rtsp && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/feilong/文档/workspace/fr/mjpg-streamer/plugins/output_rtsp/output_rtsp.c > CMakeFiles/output_rtsp.dir/output_rtsp.c.i

plugins/output_rtsp/CMakeFiles/output_rtsp.dir/output_rtsp.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/output_rtsp.dir/output_rtsp.c.s"
	cd /home/feilong/文档/workspace/fr/mjpg-streamer/cmake-build-debug/plugins/output_rtsp && /usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/feilong/文档/workspace/fr/mjpg-streamer/plugins/output_rtsp/output_rtsp.c -o CMakeFiles/output_rtsp.dir/output_rtsp.c.s

plugins/output_rtsp/CMakeFiles/output_rtsp.dir/output_rtsp.c.o.requires:

.PHONY : plugins/output_rtsp/CMakeFiles/output_rtsp.dir/output_rtsp.c.o.requires

plugins/output_rtsp/CMakeFiles/output_rtsp.dir/output_rtsp.c.o.provides: plugins/output_rtsp/CMakeFiles/output_rtsp.dir/output_rtsp.c.o.requires
	$(MAKE) -f plugins/output_rtsp/CMakeFiles/output_rtsp.dir/build.make plugins/output_rtsp/CMakeFiles/output_rtsp.dir/output_rtsp.c.o.provides.build
.PHONY : plugins/output_rtsp/CMakeFiles/output_rtsp.dir/output_rtsp.c.o.provides

plugins/output_rtsp/CMakeFiles/output_rtsp.dir/output_rtsp.c.o.provides.build: plugins/output_rtsp/CMakeFiles/output_rtsp.dir/output_rtsp.c.o


# Object files for target output_rtsp
output_rtsp_OBJECTS = \
"CMakeFiles/output_rtsp.dir/output_rtsp.c.o"

# External object files for target output_rtsp
output_rtsp_EXTERNAL_OBJECTS =

plugins/output_rtsp/output_rtsp.so: plugins/output_rtsp/CMakeFiles/output_rtsp.dir/output_rtsp.c.o
plugins/output_rtsp/output_rtsp.so: plugins/output_rtsp/CMakeFiles/output_rtsp.dir/build.make
plugins/output_rtsp/output_rtsp.so: plugins/output_rtsp/CMakeFiles/output_rtsp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/feilong/文档/workspace/fr/mjpg-streamer/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C shared library output_rtsp.so"
	cd /home/feilong/文档/workspace/fr/mjpg-streamer/cmake-build-debug/plugins/output_rtsp && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/output_rtsp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
plugins/output_rtsp/CMakeFiles/output_rtsp.dir/build: plugins/output_rtsp/output_rtsp.so

.PHONY : plugins/output_rtsp/CMakeFiles/output_rtsp.dir/build

plugins/output_rtsp/CMakeFiles/output_rtsp.dir/requires: plugins/output_rtsp/CMakeFiles/output_rtsp.dir/output_rtsp.c.o.requires

.PHONY : plugins/output_rtsp/CMakeFiles/output_rtsp.dir/requires

plugins/output_rtsp/CMakeFiles/output_rtsp.dir/clean:
	cd /home/feilong/文档/workspace/fr/mjpg-streamer/cmake-build-debug/plugins/output_rtsp && $(CMAKE_COMMAND) -P CMakeFiles/output_rtsp.dir/cmake_clean.cmake
.PHONY : plugins/output_rtsp/CMakeFiles/output_rtsp.dir/clean

plugins/output_rtsp/CMakeFiles/output_rtsp.dir/depend:
	cd /home/feilong/文档/workspace/fr/mjpg-streamer/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/feilong/文档/workspace/fr/mjpg-streamer /home/feilong/文档/workspace/fr/mjpg-streamer/plugins/output_rtsp /home/feilong/文档/workspace/fr/mjpg-streamer/cmake-build-debug /home/feilong/文档/workspace/fr/mjpg-streamer/cmake-build-debug/plugins/output_rtsp /home/feilong/文档/workspace/fr/mjpg-streamer/cmake-build-debug/plugins/output_rtsp/CMakeFiles/output_rtsp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : plugins/output_rtsp/CMakeFiles/output_rtsp.dir/depend

