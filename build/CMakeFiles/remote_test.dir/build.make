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
CMAKE_SOURCE_DIR = /home/yang/ws_firmware/src/firmware_servo_driver

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yang/ws_firmware/src/build

# Include any dependencies generated for this target.
include CMakeFiles/remote_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/remote_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/remote_test.dir/flags.make

CMakeFiles/remote_test.dir/src/remote_test.cpp.o: CMakeFiles/remote_test.dir/flags.make
CMakeFiles/remote_test.dir/src/remote_test.cpp.o: /home/yang/ws_firmware/src/firmware_servo_driver/src/remote_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yang/ws_firmware/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/remote_test.dir/src/remote_test.cpp.o"
	/bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/remote_test.dir/src/remote_test.cpp.o -c /home/yang/ws_firmware/src/firmware_servo_driver/src/remote_test.cpp

CMakeFiles/remote_test.dir/src/remote_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/remote_test.dir/src/remote_test.cpp.i"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yang/ws_firmware/src/firmware_servo_driver/src/remote_test.cpp > CMakeFiles/remote_test.dir/src/remote_test.cpp.i

CMakeFiles/remote_test.dir/src/remote_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/remote_test.dir/src/remote_test.cpp.s"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yang/ws_firmware/src/firmware_servo_driver/src/remote_test.cpp -o CMakeFiles/remote_test.dir/src/remote_test.cpp.s

# Object files for target remote_test
remote_test_OBJECTS = \
"CMakeFiles/remote_test.dir/src/remote_test.cpp.o"

# External object files for target remote_test
remote_test_EXTERNAL_OBJECTS =

remote_test: CMakeFiles/remote_test.dir/src/remote_test.cpp.o
remote_test: CMakeFiles/remote_test.dir/build.make
remote_test: /usr/local/lib/libpigpio.so
remote_test: /usr/local/lib/libpigpiod_if.so
remote_test: /usr/local/lib/libpigpiod_if2.so
remote_test: /usr/local/lib/libpigpiod_if.so
remote_test: libfirmware_servo_driver.a
remote_test: /usr/local/lib/libpigpio.so
remote_test: /usr/local/lib/libpigpiod_if.so
remote_test: /usr/local/lib/libpigpiod_if2.so
remote_test: /usr/local/lib/libpigpiod_if.so
remote_test: /usr/local/lib/libpigpiod_if2.so
remote_test: CMakeFiles/remote_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yang/ws_firmware/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable remote_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/remote_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/remote_test.dir/build: remote_test

.PHONY : CMakeFiles/remote_test.dir/build

CMakeFiles/remote_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/remote_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/remote_test.dir/clean

CMakeFiles/remote_test.dir/depend:
	cd /home/yang/ws_firmware/src/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yang/ws_firmware/src/firmware_servo_driver /home/yang/ws_firmware/src/firmware_servo_driver /home/yang/ws_firmware/src/build /home/yang/ws_firmware/src/build /home/yang/ws_firmware/src/build/CMakeFiles/remote_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/remote_test.dir/depend

