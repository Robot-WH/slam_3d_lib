/**
 * @file registration_base.hpp
 * @brief 
 * @author lwh
 * @version 1.0
 * @date 2023-03-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once 
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <eigen3/Eigen/Dense>
#include "SlamLib/Common/pointcloud.h"
#include "../../Map/TimedSlidingLocalMap.hpp"

namespace SlamLib { 
namespace pointcloud {

template<typename _PointType>
class RegistrationBase {
public:
    using MapPtr = typename map::PointCloudLocalMapBase<_PointType>::Ptr;  
    using PointVector = std::vector<_PointType, Eigen::aligned_allocator<_PointType>>;
    using Ptr = std::unique_ptr<RegistrationBase<_PointType>>;
    // 每个点匹配后的信息
    struct pointRegistrationResult {
        std::vector<double> residuals_;   // 匹配的残差
        std::vector<PointVector> nearly_points_;    // 邻居点 
    };  
    using RegistrationResult = std::unordered_map<std::string, pointRegistrationResult>;

    virtual void SetInputSource(FeaturePointCloudContainer<_PointType> const& source_input) = 0;  
    virtual void SetInputSource(std::pair<std::string, PCLConstPtr<_PointType>> const& source_input) {}
    virtual void SetInputTarget(FeaturePointCloudContainer<_PointType> const& target_input)  = 0;  
    virtual void SetInputTarget(MapPtr const& target_input) = 0;
    virtual void SetMaxIteration(uint16_t const& n) = 0;
    virtual void SetNormIteration(uint16_t const& n) = 0; 
    virtual bool Solve(Eigen::Isometry3d &T) = 0; 
    virtual RegistrationResult const& GetRegistrationResult() const = 0;
    virtual std::vector<std::string> GetUsedPointsName() = 0;  
    virtual bool HasConverged() {}
protected:
    std::vector<std::string> used_name_;   // 指定使用的点云标识名
    std::unordered_map<std::string, PCLConstPtr<_PointType>> target_features_;  // 匹配目标特征数据
}; // class LineSurfFeatureRegistration 
} // namespace
}
