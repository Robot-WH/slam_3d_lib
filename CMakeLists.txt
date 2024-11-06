cmake_minimum_required(VERSION 3.10)
project(slam_lib_3d)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)   # Debug
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Ceres REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(OpenCV 3.0 QUIET)
find_package(glog  REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_subdirectory(thirdpart/ndt_omp)

file(GLOB_RECURSE pointcloud_src CONFIGURE_DEPENDS src/PointCloud/*.cpp) 

if($ENV{ROS_VERSION} EQUAL 1)
    add_library(slam_pointcloud SHARED ${pointcloud_src})
    target_include_directories(slam_pointcloud PUBLIC 
        include
        ${PCL_INCLUDE_DIRS}
    )

    target_link_libraries(slam_pointcloud PUBLIC 
        ${PCL_LIBRARIES} 
        ${CERES_LIBRARIES}
        /usr/local/lib/libtbb.so
        /usr/local/lib/libtbbmalloc.so
        /usr/local/lib/libtbbmalloc_proxy.so
        ndt_omp
    )

    install(
        DIRECTORY include/
        DESTINATION include
    )

    # 注册 导出库文件
    install(
        TARGETS slam_pointcloud # 告诉ros2有这么个目标（可执行文件或者库）
        EXPORT slam_pointcloud
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
    )
else()
    find_package(TBB REQUIRED)
    # find_package(ament_cmake REQUIRED)
    add_library(slam_pointcloud SHARED ${pointcloud_src})
    target_include_directories(slam_pointcloud PUBLIC 
        include
        ${PCL_INCLUDE_DIRS}
        ${NDT_OMP_INCLUDE_DIR}
    )
    # message(FATAL_ERROR "NDT_OMP_INCLUDE_DIR: " ${NDT_OMP_INCLUDE_DIR})
    # ament_target_dependencies(slam_pointcloud PUBLIC ndt_omp)
    target_link_libraries(slam_pointcloud PUBLIC 
        ${PCL_LIBRARIES} 
        ${CERES_LIBRARIES}
        ${TBB_LIBRARIES}
        ndt_omp
    )

    install(
        DIRECTORY include/
        DESTINATION include
    )

    # 注册 导出库文件
    install(
        TARGETS slam_pointcloud # 告诉ros2有这么个目标（可执行文件或者库）
        EXPORT slam_pointcloud
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
    )
    # # 这里的include是Install下的include  
    # ament_export_include_directories(include ${NDT_OMP_INCLUDE_DIR})   
    # ament_export_libraries(slam_pointcloud)  
    # # ament_export_dependencies(
    # # )
    # ament_package()
endif()
