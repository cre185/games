# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /mnt/d/GAMES/GAMES202/hw4/lut-gen

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/d/GAMES/GAMES202/hw4/lut-gen/build

# Include any dependencies generated for this target.
include CMakeFiles/lut-Emu-IS.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/lut-Emu-IS.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/lut-Emu-IS.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lut-Emu-IS.dir/flags.make

CMakeFiles/lut-Emu-IS.dir/Emu_IS.cpp.o: CMakeFiles/lut-Emu-IS.dir/flags.make
CMakeFiles/lut-Emu-IS.dir/Emu_IS.cpp.o: /mnt/d/GAMES/GAMES202/hw4/lut-gen/Emu_IS.cpp
CMakeFiles/lut-Emu-IS.dir/Emu_IS.cpp.o: CMakeFiles/lut-Emu-IS.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/d/GAMES/GAMES202/hw4/lut-gen/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/lut-Emu-IS.dir/Emu_IS.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/lut-Emu-IS.dir/Emu_IS.cpp.o -MF CMakeFiles/lut-Emu-IS.dir/Emu_IS.cpp.o.d -o CMakeFiles/lut-Emu-IS.dir/Emu_IS.cpp.o -c /mnt/d/GAMES/GAMES202/hw4/lut-gen/Emu_IS.cpp

CMakeFiles/lut-Emu-IS.dir/Emu_IS.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lut-Emu-IS.dir/Emu_IS.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/d/GAMES/GAMES202/hw4/lut-gen/Emu_IS.cpp > CMakeFiles/lut-Emu-IS.dir/Emu_IS.cpp.i

CMakeFiles/lut-Emu-IS.dir/Emu_IS.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lut-Emu-IS.dir/Emu_IS.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/d/GAMES/GAMES202/hw4/lut-gen/Emu_IS.cpp -o CMakeFiles/lut-Emu-IS.dir/Emu_IS.cpp.s

# Object files for target lut-Emu-IS
lut__Emu__IS_OBJECTS = \
"CMakeFiles/lut-Emu-IS.dir/Emu_IS.cpp.o"

# External object files for target lut-Emu-IS
lut__Emu__IS_EXTERNAL_OBJECTS =

lut-Emu-IS: CMakeFiles/lut-Emu-IS.dir/Emu_IS.cpp.o
lut-Emu-IS: CMakeFiles/lut-Emu-IS.dir/build.make
lut-Emu-IS: CMakeFiles/lut-Emu-IS.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/d/GAMES/GAMES202/hw4/lut-gen/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable lut-Emu-IS"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lut-Emu-IS.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lut-Emu-IS.dir/build: lut-Emu-IS
.PHONY : CMakeFiles/lut-Emu-IS.dir/build

CMakeFiles/lut-Emu-IS.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lut-Emu-IS.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lut-Emu-IS.dir/clean

CMakeFiles/lut-Emu-IS.dir/depend:
	cd /mnt/d/GAMES/GAMES202/hw4/lut-gen/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/d/GAMES/GAMES202/hw4/lut-gen /mnt/d/GAMES/GAMES202/hw4/lut-gen /mnt/d/GAMES/GAMES202/hw4/lut-gen/build /mnt/d/GAMES/GAMES202/hw4/lut-gen/build /mnt/d/GAMES/GAMES202/hw4/lut-gen/build/CMakeFiles/lut-Emu-IS.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lut-Emu-IS.dir/depend
