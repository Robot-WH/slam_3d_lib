#include "SlamLib/PointCloud/Filter/filter_base.h"
namespace SlamLib {
namespace pointcloud {
/**
 * @brief:  滤波流程
 * @param[in] cloud_in 输入的点云 
 * @param[out] cloud_out 处理后的点云 
 */        
template<typename _PointType>
void FilterBase<_PointType>::Filter(const PointCloudConstPtr& cloud_in, 
                                                                            PointCloudPtr& cloud_out) const {
    if (filter_ptr_ == nullptr)
        return; 
    filter_ptr_->setInputCloud(cloud_in);
    cloud_out.reset(new pcl::PointCloud<_PointType>);
    filter_ptr_->filter(*cloud_out);
    cloud_out->header = cloud_in->header;
    return;  
}

template<typename _PointType>
void FilterBase<_PointType>::Filter(PointCloudPtr& cloud_in) const {
    if (filter_ptr_ == nullptr)
        return; 
    filter_ptr_->setInputCloud(cloud_in);
    filter_ptr_->filter(*cloud_in);
    return;  
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
