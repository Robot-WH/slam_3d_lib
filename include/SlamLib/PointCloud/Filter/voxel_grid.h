
#pragma once 
#define PCL_NO_PRECOMPILE
#include "filter_base.h"
#include <pcl/filters/voxel_grid.h>

namespace SlamLib {
namespace pointcloud {

template<class _PointT>
using  VoxelGridPtr = std::unique_ptr<pcl::VoxelGrid<_PointT>>;  

struct VoxelGridOption {
    float resolution_ = 0;      // 滤波器体素的直径
};
struct ApproximateVoxelGridOption {
};

template<typename _PointType>
class VoxelGridFilter : public FilterBase<_PointType> {
public:
    struct Option {
        std::string mode_;
        VoxelGridOption voxel_grid_option_;
        ApproximateVoxelGridOption approximate_voxel_grid_option_;
    }; 
    VoxelGridFilter() {}  
    VoxelGridFilter(Option option);
    void Reset(Option option);
};
} // namespace 
} // namespace 

