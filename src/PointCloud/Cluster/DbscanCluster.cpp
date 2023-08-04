
#pragma once 
// #define PCL_NO_PRECOMPILE
#include "SlamLib/PointCloud/Cluster/DbscanCluster_impl.hpp"
#include "SlamLib/Common/point_type.h"

namespace SlamLib {
namespace pointcloud {

template class DBSCANCluster<PointXYZIRDT>; 
template class DBSCANCluster<PointXYZIRDTC>; 
}
}