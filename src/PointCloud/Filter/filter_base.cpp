#include "SlamLib/PointCloud/Filter/filter_base.h"

namespace SlamLib {
namespace pointcloud {

/**
 * @brief:  滤波流程
 * @param[in] cloud_in 输入的点云 
 * @param[out] cloud_out 处理后的点云 
 */        
template<typename _PointType>
typename FilterBase<_PointType>::PointCloudPtr 
FilterBase<_PointType>::Filter(const PointCloudConstPtr& cloud_in) const {
    PointCloudPtr cloud_out(
        new pcl::PointCloud<_PointType>(*cloud_in));  
    if (filter_ptr_ == nullptr)
        return cloud_out; 
    filter_ptr_->setInputCloud(cloud_in);
    filter_ptr_->filter(*cloud_out);
    cloud_out->header = cloud_in->header;
    return cloud_out;  
}

/**
 * @brief: 设置滤波器   通过这个就能更换滤波器
 */        
template<typename _PointType>
void FilterBase<_PointType>::SetFilter(typename pcl::Filter<_PointType>::Ptr const& filter_ptr) {
    filter_ptr_ = filter_ptr;   
}

template class FilterBase<pcl::PointXYZI>; 
template class FilterBase<PointXYZIRDT>; 
template class FilterBase<PointXYZIRDTC>; 
}; // class FilterBase
}
