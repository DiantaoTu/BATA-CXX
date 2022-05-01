cmake_minimum_required(VERSION 3.4)
project(BATA)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-fopenmp")
# set(CMAKE_BUILD_TYPE "Debug")
IF (CMAKE_BUILD_TYPE MATCHES "Debug")
        MESSAGE(STATUS "CMAKE_BUILD_TYPE = Debug")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
ELSEIF (CMAKE_BUILD_TYPE MATCHES "Release")
        MESSAGE(STATUS "CMAKE_BUILD_TYPE = Release")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
ELSE()
        set(CMAKE_BUILD_TYPE "Release")
        MESSAGE(STATUS "Set defalult CMAKE_BUILD_TYPE = Release")
ENDIF()


INCLUDE_DIRECTORIES("/usr/local/include/eigen3/")

SET(LIBRARIES
        boost_filesystem
        glog
        )
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "find opencv version:" ${OpenCV_VERSION})

aux_source_directory(. source_file)
add_executable(BATA ${source_file})
target_link_libraries(BATA ${LIBRARIES} )
