cmake_minimum_required(VERSION 3.0.2)
project(slam_lib_3d)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)   # Debug
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Ceres REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(OpenCV 3.0 QUIET)
find_package(GTSAM REQUIRED QUIET)
find_package(glog  REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(Slamlib_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE PATH 
    "project include location" FORCE)
# message("SlamLib_INCLUDE_DIR: " ${SlamLib_INCLUDE_DIR})

# GLOB_RECURSE 会递归的找到子目录文件
# 加了CONFIGURE_DEPENDS，如果有新文件添加，cmake --build 会自动运行 cmake -B build 进行更新
file(GLOB_RECURSE pointcloud_src CONFIGURE_DEPENDS src/PointCloud/*.cpp) 

add_library(slam_pointcloud SHARED ${pointcloud_src})
target_include_directories(slam_pointcloud PUBLIC 
    include
    ${PCL_INCLUDE_DIRS}
    ${NDT_OMP_INCLUDE_DIR}
)
target_link_libraries(slam_pointcloud PUBLIC 
    ${PCL_LIBRARIES} 
    ${CERES_LIBRARIES}
    ndt_omp
    /usr/local/lib/libtbb.so
    /usr/local/lib/libtbbmalloc.so
    /usr/local/lib/libtbbmalloc_proxy.so
)
