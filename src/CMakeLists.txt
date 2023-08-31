cmake_minimum_required(VERSION 3.5)

project(LidarDenoising LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(LidarDenoising main.cpp denoising.hpp kdtree.hpp PointCloud.hpp)

target_link_libraries(LidarDenoising ${PCL_LIBRARIES})

install(TARGETS LidarDenoising
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

