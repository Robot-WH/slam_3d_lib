#include "SlamLib/PointCloud/Registration/edgeSurfFeatureRegistration_impl.hpp"
#include "SlamLib/Common/point_type.h"
namespace SlamLib { 
namespace pointcloud {

template class EdgeSurfFeatureRegistration<pcl::PointXYZI>; 
template class EdgeSurfFeatureRegistration<PointXYZIRDT>; 
template class EdgeSurfFeatureRegistration<PointXYZIRDTC>; 
}
}

