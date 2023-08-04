/*
 * @Copyright(C): Your Company
 * @FileName: 文件名
 * @Author: 作者
 * @Version: 版本
 * @Date: 2022-04-28 23:22:00
 * @Description: 
 * @Others: 
 */

#pragma once 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * 6D位姿点云结构定义
*/
struct PointXYZIRPYT {
    PCL_ADD_POINT4D     
    PCL_ADD_INTENSITY;  
    float roll;         
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
} EIGEN_ALIGN16;                    

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                (float, x, x) (float, y, y)
                                (float, z, z) (float, intensity, intensity)
                                (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                (double, time, time));

typedef PointXYZIRPYT  PointTypePose;

/**
 * XYZI + ring + time +range 点云结构
*/
struct PointXYZIRDT {
    PCL_ADD_POINT4D     // 位置
    PCL_ADD_INTENSITY;  // 激光点反射强度，也可以存点的索引
    int16_t ring = -1;      // 扫描线
    float time = -1;         // 时间戳，记录相对于当前帧第一个激光点的时差，第一个点time=0
    float range = 0;   // 深度 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;        // 内存16字节对齐，EIGEN SSE优化要求

// 注册为PCL点云格式
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRDT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (int16_t, ring, ring) (float, time, time) (float, range, range)
);

/**
 *  点云结构
*/
struct PointXYZIRDTC {
    PCL_ADD_POINT4D     // 位置
    PCL_ADD_INTENSITY;  // 激光点反射强度，也可以存点的索引
    int16_t ring = -1;      // 扫描线
    float time = -1;         // 时间戳，记录相对于当前帧第一个激光点的时差，第一个点time=0
    float range = 0;   // 深度 
    uint8_t type = 0;   // 点的类型    1：稳定点    2、动态点     3、地面点 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;        // 内存16字节对齐，EIGEN SSE优化要求

// 注册为PCL点云格式
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRDTC,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (int16_t, ring, ring) (float, time, time) (float, range, range)
    (uint8_t, type, type)
);

// ouster 激光点云格式
struct EIGEN_ALIGN16 ousterPoint {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ousterPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)(std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)(std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range));

// velodyne 激光点云格式
struct EIGEN_ALIGN16 velodynePoint {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(velodynePoint,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (float, time, time)(std::uint16_t, ring, ring));