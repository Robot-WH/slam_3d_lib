
#include "SlamLib/PointCloud/Filter/voxel_grid.h"
#include "SlamLib/PointCloud/Filter/outlier_removal.h"

int main() {
    SlamLib::pointcloud::VoxelGridFilter<PointXYZIRPYT> vg; 
    SlamLib::pointcloud::OutlierRemovalFilter<PointXYZIRPYT> rm;
    
    return 0; 
}