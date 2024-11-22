cmake_minimum_required(VERSION 3.5)

# Global router project
project(MazeRouter)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set compiler warnings
include(cmake/CompilerWarnings.cmake)
gr_set_project_warnings()

# Set compiler flags for performance
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fomit-frame-pointer -O3 -march=native -mtune=native -DNDEBUG")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -O0 --analyze")

# Create compile commands for clangd to look for
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add executable targets
add_executable(SimpleGR src/SimpleGR.cpp src/IO.cpp src/Utils.cpp src/main.cpp src/MazeRouter.cpp)
add_executable(mapper src/IO.cpp src/Utils.cpp src/mapper.cpp)
