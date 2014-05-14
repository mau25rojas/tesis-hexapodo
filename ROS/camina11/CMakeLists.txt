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


rosbuild_add_boost_directories()
#rosbuild_add_library(Nodo1_datosTrayectoriaPata ~/fuerte_workspace/sandbox/TesisMaureen/ROS/Convexhull/vector3d.cpp)
rosbuild_add_executable(Nodo1_datosTrayectoriaPata src/Nodo1_datosTrayectoriaPata.cpp src/constantes.cpp ../../Convexhull/vector3d.cpp)
rosbuild_add_executable(Nodo2_parametrizacion src/Nodo2_parametrizacion.cpp src/constantes.cpp ../../Convexhull/vector3d.cpp)
rosbuild_add_executable(Nodo3_RosVrep src/Nodo3_RosVrep.cpp src/constantes.cpp ../../Convexhull/vector3d.cpp)
#rosbuild_add_executable(Nodo5_ConstruccionDeMapa src/Nodo5_ConstruccionDeMapa.cpp src/constantes.cpp ../../Convexhull/vector3d.cpp)
#target_link_libraries(Nodo5_ConstruccionDeMapa -L/usr/lib -Wl,-Bsymbolic-functions -lalleg-4.2.2)
rosbuild_add_executable(Nodo6_UbicacionRobot src/Nodo6_UbicacionRobot2.cpp src/constantes.cpp ../../Convexhull/vector3d.cpp)
#rosbuild_add_executable(CentroMasa src/CentrodeMasa.cpp src/constantes.cpp ../../Convexhull/vector3d.cpp ../../Convexhull/analisis.cpp ../../Convexhull/convexhull.cpp)
#target_link_libraries(CentroMasa -L/usr/lib -Wl,-Bsymbolic-functions -lalleg-4.2.2)

#CLientes y servicios
rosbuild_add_executable(server_Cinversa src/server_Cinversa.cpp)
#rosbuild_add_library(server_PlanificadorPisada ~/fuerte_workspace/sandbox/TesisMaureen/ROS/Convexhull/vector3d.cpp)
#rosbuild_add_executable(server_PlanificadorPisada src/server_PlanificadorPisada.cpp)



