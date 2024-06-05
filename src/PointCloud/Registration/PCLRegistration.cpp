/**
 * @file ndtompRegistration.hpp
 * @brief 
 * @author lwh
 * @version 1.0
 * 
 * @copyright Copyright (c) 2022
 */
#include "SlamLib/PointCloud/Registration/PCLRegistration_impl.hpp"
#include "SlamLib/Common/point_type.h"

namespace SlamLib { 
namespace pointcloud {
template class PCLRegistration<PointXYZIRDT>; 
template class PCLRegistration<pcl::PointXYZI>; 
template class PCLRegistration<PointXYZIRDTC>; 
} // namespace
}
