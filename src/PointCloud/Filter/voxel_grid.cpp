
#include "SlamLib/PointCloud/Filter/voxel_grid.h"

namespace SlamLib {
namespace pointcloud {

template<typename _PointType>
VoxelGridFilter<_PointType>::VoxelGridFilter(FilterOption::VoxelGridFilterOption option) {
    Reset(option); 
}

/**
 * @brief: 对降采样的模式进行设置
 * @param mode 滤波器的模式 
 * @return {*}
 */            
template<typename _PointType>
void VoxelGridFilter<_PointType>::Reset(FilterOption::VoxelGridFilterOption option) {
    if (option.mode_ == "VoxelGrid") {
        // 用体素内点云的重心代替原点云   
        VoxelGridPtr<_PointType> voxelgrid(new pcl::VoxelGrid<_PointType>());
        voxelgrid->setLeafSize(option.voxel_grid_option_.resolution_, 
                                                        option.voxel_grid_option_.resolution_, 
                                                        option.voxel_grid_option_.resolution_);
        this->SetFilter(std::move(voxelgrid)); 
    } else if (option.mode_ == "ApproximateVoxelGrid") {
    }
}

template class VoxelGridFilter<pcl::PointXYZI>; 
template class VoxelGridFilter<PointXYZIRDT>; 
template class VoxelGridFilter<PointXYZIRDTC>; 
} // namespace 
} // namespace 

