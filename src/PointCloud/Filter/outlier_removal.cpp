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
#include "SlamLib/PointCloud/Filter/outlier_removal.h"

namespace SlamLib {
namespace pointcloud {

template<typename _PointType>
OutlierRemovalFilter<_PointType>::OutlierRemovalFilter(Option option) {
    Reset(option);
}

/**
 * @brief  重新设置滤波器 
 * 
 * @param option 
 */
template<typename _PointType>
void OutlierRemovalFilter<_PointType>::Reset(Option option) {
    if (option.mode_ == "radiusOutlier") {
        RadiusOutlierRemovalPtr<_PointType> rad(new pcl::RadiusOutlierRemoval<_PointType>());
        rad->setRadiusSearch(option.radiusOutlier_option_.radius_);                                         
        rad->setMinNeighborsInRadius(option.radiusOutlier_option_.min_neighbors_);  
        this->SetFilter(std::move(rad)); 
    } else if (option.mode_ == "statisticalOutlier") {
        StatisticalOutlierRemovalPtr<_PointType> sor(new pcl::StatisticalOutlierRemoval<_PointType>());
        sor->setMeanK (option.statisticalOutlier_option_.mean_k_);    
        sor->setStddevMulThresh (option.statisticalOutlier_option_.k_);
        this->SetFilter(std::move(sor)); 
    }
}

template class OutlierRemovalFilter<pcl::PointXYZI>; 
template class OutlierRemovalFilter<PointXYZIRDT>; 
template class OutlierRemovalFilter<PointXYZIRDTC>; 
} // namespace 
} 
