# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/johannes/dev/BaboCam2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/johannes/dev/BaboCam2

# Include any dependencies generated for this target.
include CMakeFiles/BaboCam.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/BaboCam.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/BaboCam.dir/flags.make

CMakeFiles/BaboCam.dir/src/BaboCam.cpp.o: CMakeFiles/BaboCam.dir/flags.make
CMakeFiles/BaboCam.dir/src/BaboCam.cpp.o: src/BaboCam.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/johannes/dev/BaboCam2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/BaboCam.dir/src/BaboCam.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/BaboCam.dir/src/BaboCam.cpp.o -c /home/johannes/dev/BaboCam2/src/BaboCam.cpp

CMakeFiles/BaboCam.dir/src/BaboCam.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BaboCam.dir/src/BaboCam.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/johannes/dev/BaboCam2/src/BaboCam.cpp > CMakeFiles/BaboCam.dir/src/BaboCam.cpp.i

CMakeFiles/BaboCam.dir/src/BaboCam.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BaboCam.dir/src/BaboCam.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/johannes/dev/BaboCam2/src/BaboCam.cpp -o CMakeFiles/BaboCam.dir/src/BaboCam.cpp.s

CMakeFiles/BaboCam.dir/src/BaboCam.cpp.o.requires:

.PHONY : CMakeFiles/BaboCam.dir/src/BaboCam.cpp.o.requires

CMakeFiles/BaboCam.dir/src/BaboCam.cpp.o.provides: CMakeFiles/BaboCam.dir/src/BaboCam.cpp.o.requires
	$(MAKE) -f CMakeFiles/BaboCam.dir/build.make CMakeFiles/BaboCam.dir/src/BaboCam.cpp.o.provides.build
.PHONY : CMakeFiles/BaboCam.dir/src/BaboCam.cpp.o.provides

CMakeFiles/BaboCam.dir/src/BaboCam.cpp.o.provides.build: CMakeFiles/BaboCam.dir/src/BaboCam.cpp.o


CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.o: CMakeFiles/BaboCam.dir/flags.make
CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.o: src/socket/ControlSocket.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/johannes/dev/BaboCam2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.o -c /home/johannes/dev/BaboCam2/src/socket/ControlSocket.cpp

CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/johannes/dev/BaboCam2/src/socket/ControlSocket.cpp > CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.i

CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/johannes/dev/BaboCam2/src/socket/ControlSocket.cpp -o CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.s

CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.o.requires:

.PHONY : CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.o.requires

CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.o.provides: CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.o.requires
	$(MAKE) -f CMakeFiles/BaboCam.dir/build.make CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.o.provides.build
.PHONY : CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.o.provides

CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.o.provides.build: CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.o


CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.o: CMakeFiles/BaboCam.dir/flags.make
CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.o: src/color/ColorStreamThread.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/johannes/dev/BaboCam2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.o -c /home/johannes/dev/BaboCam2/src/color/ColorStreamThread.cpp

CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/johannes/dev/BaboCam2/src/color/ColorStreamThread.cpp > CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.i

CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/johannes/dev/BaboCam2/src/color/ColorStreamThread.cpp -o CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.s

CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.o.requires:

.PHONY : CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.o.requires

CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.o.provides: CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.o.requires
	$(MAKE) -f CMakeFiles/BaboCam.dir/build.make CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.o.provides.build
.PHONY : CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.o.provides

CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.o.provides.build: CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.o


# Object files for target BaboCam
BaboCam_OBJECTS = \
"CMakeFiles/BaboCam.dir/src/BaboCam.cpp.o" \
"CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.o" \
"CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.o"

# External object files for target BaboCam
BaboCam_EXTERNAL_OBJECTS =

BaboCam: CMakeFiles/BaboCam.dir/src/BaboCam.cpp.o
BaboCam: CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.o
BaboCam: CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.o
BaboCam: CMakeFiles/BaboCam.dir/build.make
BaboCam: /usr/local/lib/libopencv_dnn.so.4.0.1
BaboCam: /usr/local/lib/libopencv_gapi.so.4.0.1
BaboCam: /usr/local/lib/libopencv_ml.so.4.0.1
BaboCam: /usr/local/lib/libopencv_objdetect.so.4.0.1
BaboCam: /usr/local/lib/libopencv_photo.so.4.0.1
BaboCam: /usr/local/lib/libopencv_stitching.so.4.0.1
BaboCam: /usr/local/lib/libopencv_video.so.4.0.1
BaboCam: /usr/local/lib/libopencv_calib3d.so.4.0.1
BaboCam: /usr/local/lib/libopencv_features2d.so.4.0.1
BaboCam: /usr/local/lib/libopencv_flann.so.4.0.1
BaboCam: /usr/local/lib/libopencv_highgui.so.4.0.1
BaboCam: /usr/local/lib/libopencv_videoio.so.4.0.1
BaboCam: /usr/local/lib/libopencv_imgcodecs.so.4.0.1
BaboCam: /usr/local/lib/libopencv_imgproc.so.4.0.1
BaboCam: /usr/local/lib/libopencv_core.so.4.0.1
BaboCam: CMakeFiles/BaboCam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/johannes/dev/BaboCam2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable BaboCam"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/BaboCam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/BaboCam.dir/build: BaboCam

.PHONY : CMakeFiles/BaboCam.dir/build

CMakeFiles/BaboCam.dir/requires: CMakeFiles/BaboCam.dir/src/BaboCam.cpp.o.requires
CMakeFiles/BaboCam.dir/requires: CMakeFiles/BaboCam.dir/src/socket/ControlSocket.cpp.o.requires
CMakeFiles/BaboCam.dir/requires: CMakeFiles/BaboCam.dir/src/color/ColorStreamThread.cpp.o.requires

.PHONY : CMakeFiles/BaboCam.dir/requires

CMakeFiles/BaboCam.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/BaboCam.dir/cmake_clean.cmake
.PHONY : CMakeFiles/BaboCam.dir/clean

CMakeFiles/BaboCam.dir/depend:
	cd /home/johannes/dev/BaboCam2 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/johannes/dev/BaboCam2 /home/johannes/dev/BaboCam2 /home/johannes/dev/BaboCam2 /home/johannes/dev/BaboCam2 /home/johannes/dev/BaboCam2/CMakeFiles/BaboCam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/BaboCam.dir/depend

