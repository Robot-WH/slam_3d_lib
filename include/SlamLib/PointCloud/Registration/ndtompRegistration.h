/**
 * @file ndtompRegistration.hpp
 * @brief 
 * @author lwh
 * @version 1.0
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once 
#define PCL_NO_PRECOMPILE
#include <pcl/registration/icp.h>
#include "registration_base.h"

namespace SlamLib { 
namespace pointcloud {

template<typename _PointType>
class NdtOmpRegistration : public RegistrationBase<_PointType> {
public:
    using  RegistrationPtr = std::unique_ptr<pcl::Registration<_PointType, _PointType>>; 
    NdtOmpRegistration(RegistrationPtr ptr, std::string used_point_label);
    virtual void SetInputSource(FeaturePointCloudContainer<_PointType> 
                                                    const& source_input) override;
    virtual void SetInputSource(
        std::pair<std::string, PCLConstPtr<_PointType>> const& source_input) override;
    virtual void SetInputTarget(FeaturePointCloudContainer<_PointType> 
                                                    const& target_input) override;
    virtual void SetInputTarget(
        typename RegistrationBase<_PointType>::MapPtr const& target_input) override;
    virtual void SetMaxIteration(uint16_t const& n) override;
    virtual void SetNormIteration(uint16_t const& n) override;
    virtual bool Solve(Eigen::Isometry3d &T) override;
    virtual bool HasConverged() override;

    virtual typename RegistrationBase<_PointType>::RegistrationResult const& 
    GetRegistrationResult() const override;

    virtual std::vector<std::string> GetUsedPointsName() override;
private:
    std::string used_point_label_;
    RegistrationPtr registration_ptr_;  
}; // class LineSurfFeatureRegistration 
} // namespace
}
