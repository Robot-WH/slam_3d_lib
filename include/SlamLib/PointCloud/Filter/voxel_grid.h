/**
 * @file voxel_grid.h
 * @author lwh ()
 * @brief 
 * @version 0.1
 * @date 2023-11-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once 
#define PCL_NO_PRECOMPILE
#include "filter_base.h"
#include <pcl/filters/voxel_grid.h>
namespace SlamLib {
namespace pointcloud {

template<class _PointT>
using  VoxelGridPtr = std::unique_ptr<pcl::VoxelGrid<_PointT>>;  

namespace FilterOption {
struct VoxelGridOption {
    float resolution_ = 0;      // 滤波器体素的直径
};

struct ApproximateVoxelGridOption {
};

struct VoxelGridFilterOption {
    std::string mode_;
    VoxelGridOption voxel_grid_option_;
    ApproximateVoxelGridOption approximate_voxel_grid_option_;
}; 
}

template<typename _PointType>
class VoxelGridFilter : public FilterBase<_PointType> {
public:
    VoxelGridFilter() {}
    VoxelGridFilter(FilterOption::VoxelGridFilterOption option);
    void Reset(FilterOption::VoxelGridFilterOption option);
};
} // namespace 
} // namespace 

