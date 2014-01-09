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

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(Nodo1_datosTrayectoriaPata src/Nodo1_datosTrayectoriaPata.cpp)
rosbuild_add_executable(Nodo2_parametrizacion src/Nodo2_parametrizacion.cpp)
rosbuild_add_executable(Nodo3_RosVrep src/Nodo3_RosVrep.cpp)
#CLientes y servicios
rosbuild_add_executable(server_Cinversa src/server_Cinversa.cpp)
rosbuild_add_executable(server_TransHomogenea src/server_TransformacionHomogenea.cpp)
rosbuild_add_executable(server_TrayectoriaMundoPata src/server_TrayectoriaMundoPata.cpp)

#Nodo para usar el joystick FORCE3DPRO
#rosbuild_add_executable(force3D_input src/force3D_input.cpp)
#Nodo para convertir los comandos de ROS para los joints a comandos para la SSC-32
rosbuild_add_executable(arana_connection src/arana_connection.cpp)
#Nodo para pasar los comandos del SSC-32 por serial
rosbuild_add_executable(comm util/comm/comm.cpp)
rosbuild_link_boost(comm thread)
#Nodo para pasar los Q de los joints de V-rep a los Q del hexapodo
rosbuild_add_executable(vrep_to_arana src/vrep_to_arana1.cpp)
#Nodo para pasar los Q a mano a los Q del hexapodo
#rosbuild_add_executable(move_arana src/move_arana_prueba.cpp)

