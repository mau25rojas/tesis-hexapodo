cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries 
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

rosbuild_add_executable(Nodo0_controlMovimientoRob src/Nodo0_controlMovimientoRob.cpp)
rosbuild_add_library(Nodo1_datosTrayectoriaPata ~/fuerte_workspace/sandbox/TesisMaureen/ROS/Convexhull/vector3d.cpp)
rosbuild_add_executable(Nodo1_datosTrayectoriaPata src/Nodo1_datosTrayectoriaPata.cpp)
rosbuild_add_library(Nodo2_parametrizacion ~/fuerte_workspace/sandbox/TesisMaureen/ROS/Convexhull/vector3d.cpp)
rosbuild_add_executable(Nodo2_parametrizacion src/Nodo2_parametrizacion.cpp)
rosbuild_add_executable(Nodo3_RosVrep src/Nodo3_RosVrep.cpp)
rosbuild_add_executable(server_Cinversa src/server_Cinversa.cpp)

