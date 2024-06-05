/**
 * @file outlier_removal.hpp
 * @author lwh ()
 * @brief 
 * @version 0.1
 * @date 2023-06-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once 
#define PCL_NO_PRECOMPILE
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "filter_base.h"
namespace SlamLib {
namespace pointcloud {
namespace FilterOption {
struct RadiusOutlierOption {
    float radius_;  // 考虑的范围
    uint16_t min_neighbors_;  // 范围内最小邻居数  
};
struct StatisticalOutlierOption {
    uint16_t mean_k_;
    uint8_t k_;
};
struct OutlierRemovalFilterOption {
    std::string mode_;  
    RadiusOutlierOption radiusOutlier_option_; 
    StatisticalOutlierOption statisticalOutlier_option_; 
}; 
}

template<class PointT>
using RadiusOutlierRemovalPtr = std::unique_ptr<pcl::RadiusOutlierRemoval<PointT>>;  
template<class PointT>
using  StatisticalOutlierRemovalPtr = std::unique_ptr<pcl::StatisticalOutlierRemoval<PointT>>;  

template<typename _PointType>
class OutlierRemovalFilter : public FilterBase<_PointType> {
public:
    OutlierRemovalFilter() {}
    OutlierRemovalFilter(FilterOption::OutlierRemovalFilterOption option);
    void Reset(FilterOption::OutlierRemovalFilterOption option);
};
} // namepace 
} // namespace
