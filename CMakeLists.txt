cmake_minimum_required(VERSION 2.6)
project(3D)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_definitions(-std=c++11 -w)

aux_source_directory(./util util_src)
add_executable(do_3D main.cpp ${util_src})

target_link_libraries (do_3D ${PCL_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS do_3D RUNTIME DESTINATION bin)