#include "SlamLib/PointCloud/Registration/ceres_edgeSurfFeatureRegistration_impl.hpp"
#include "SlamLib/Common/point_type.h"
namespace SlamLib { 
namespace pointcloud {

template class CeresEdgeSurfFeatureRegistration<pcl::PointXYZI>; 
template class CeresEdgeSurfFeatureRegistration<PointXYZIRDT>; 
template class CeresEdgeSurfFeatureRegistration<PointXYZIRDTC>; 
}
}
