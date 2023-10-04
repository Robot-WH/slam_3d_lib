
// #define PCL_NO_PRECOMPILE
#include "SlamLib/PointCloud/Cluster/EuclideanCluster_impl.hpp"
#include "SlamLib/Common/point_type.h"

namespace SlamLib {
namespace pointcloud {

template class EuclideanCluster<PointXYZIRDT>; 
template class EuclideanCluster<PointXYZIRDTC>; 
}
}