
#pragma once 
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include "SlamLib/Common/point_type.h"

namespace SlamLib {
namespace pointcloud {
/**
 * @brief: pcl 滤波器的封装
 */
template<typename _PointType>
class FilterBase {
public:
    using PointCloudPtr = typename pcl::PointCloud<_PointType>::Ptr;  
    using PointCloudConstPtr = typename pcl::PointCloud<_PointType>::ConstPtr; 

    FilterBase() : filter_ptr_(nullptr) {}
    virtual ~FilterBase() {}
    virtual PointCloudPtr Filter(const PointCloudConstPtr& cloud_in) const;
    void SetFilter(typename pcl::Filter<_PointType>::Ptr const& filter_ptr);
    
private:
    typename pcl::Filter<_PointType>::Ptr filter_ptr_; 
}; // class FilterBase
}
}
