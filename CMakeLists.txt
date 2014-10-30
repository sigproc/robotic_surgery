cmake_minimum_required(VERSION 2.6.2)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake/Modules/)

find_package(Jsoncpp REQUIRED)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

set(includeFolders ${PROJECT_SOURCE_DIR}/include/k2_client)

set(RGBsourceFiles ${PROJECT_SOURCE_DIR}/src/startRGB.cpp)
set(IRsourceFiles ${PROJECT_SOURCE_DIR}/src/startIR.cpp)
set(DEPTHsourceFiles ${PROJECT_SOURCE_DIR}/src/startDepth.cpp)
set(AUDIOsourceFiles ${PROJECT_SOURCE_DIR}/src/startAudio.cpp)
set(BODYsourceFiles ${PROJECT_SOURCE_DIR}/src/startBody.cpp)

include_directories(${includeFolders})
include_directories(${JSONCPP_INCLUDE_DIR})

rosbuild_add_executable(startRGB ${RGBsourceFiles})
rosbuild_add_executable(startIR ${IRsourceFiles})
rosbuild_add_executable(startDepth ${DEPTHsourceFiles})
rosbuild_add_executable(startAudio ${AUDIOsourceFiles})
rosbuild_add_executable(startBody ${BODYsourceFiles})

target_link_libraries(startAudio ${JSONCPP_LIBRARY})
target_link_libraries(startBody ${JSONCPP_LIBRARY})
