cmake_minimum_required(VERSION 3.4)
project(test)

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

FIND_PACKAGE(Sophus REQUIRED)
INCLUDE_DIRECTORIES(${Sophus_INCLUDE_DIRS})

INCLUDE_DIRECTORIES("/usr/local/include/eigen3/")

find_package(PCL REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}) 
add_definitions(${PCL_DEFINITIONS})

find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIRS})

SET(LIBRARIES
        boost_filesystem
        glog
        )
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "find opencv version:" ${OpenCV_VERSION})

aux_source_directory(. source_file)
add_executable(test ${source_file})
target_link_libraries(test ${LIBRARIES} ${Sophus_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES} )
