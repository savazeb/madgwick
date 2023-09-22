cmake_minimum_required(VERSION 3.10)
project(madgwick)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

file(GLOB_RECURSE SOURCE_FILES src/*.cpp)

# Create an executable target
add_executable(madgwick ${SOURCE_FILES})

# Specify include directories for the target
target_include_directories(madgwick PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)
