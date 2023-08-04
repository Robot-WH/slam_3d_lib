
#pragma once 

#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace SlamLib {

// using type_id = uint8_t;
// using PointType = pcl::PointXYZI;
// using PCLPtr = pcl::PointCloud<PointType>::Ptr;  
// using PCLType = pcl::PointCloud<PointType>;  

template<typename _T>
using PCLConstPtr = typename pcl::PointCloud<_T>::ConstPtr;  
template<typename _T>
using PCLPtr = typename pcl::PointCloud<_T>::Ptr;  


template<typename _PointType>
using FeaturePointCloudContainer = std::unordered_map<std::string, 
        typename pcl::PointCloud<_PointType>::ConstPtr>;

/**
 * @brief:  保存特征点各种信息数据 
 * @param _FeatureType 特征点云数据类型  
 */    
template<typename _FeatureT>
struct CloudContainer {   
    double timestamp_start_ = 0;  // 激光帧起始点的时间戳 
    double timestamp_end_ = 0;   // 激光帧最后一个点的时间戳
    uint32_t ori_points_num = 0;  
    // 用于匹配的特征点云
    FeaturePointCloudContainer<_FeatureT> feature_data_;       
    // 点类数据      <数据标识名，数据体>
    FeaturePointCloudContainer<_FeatureT> pointcloud_data_;     
}; 
}
