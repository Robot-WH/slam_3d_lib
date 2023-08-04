/**
 * @file ndtompRegistration.hpp
 * @brief 
 * @author lwh
 * @version 1.0
 * 
 * @copyright Copyright (c) 2022
 */
#include "SlamLib/PointCloud/Registration/ndtompRegistration_impl.hpp"
#include "SlamLib/Common/point_type.h"

namespace SlamLib { 
namespace pointcloud {

template class NdtOmpRegistration<PointXYZIRDT>; 
template class NdtOmpRegistration<pcl::PointXYZI>; 
template class NdtOmpRegistration<PointXYZIRDTC>; 
} // namespace
}
