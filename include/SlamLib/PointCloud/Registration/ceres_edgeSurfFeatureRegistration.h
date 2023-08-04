#pragma once 

#include "registration_base.h"
#include "SlamLib/Ceres/ceres_factor/edge_factor.hpp"
#include "SlamLib/Ceres/ceres_factor/surf_factor.hpp"
#include "SlamLib/Ceres/Parameterization/PoseSE3Parameterization.hpp"
#include "SlamLib/tic_toc.hpp"
#include "FeatureMatch/EdgeFeatureMatch.hpp"
#include "FeatureMatch/surfFeatureMatch.hpp"
#include <execution>  // C++ 17 并行算法 
#include <mutex>
#include <atomic>

namespace SlamLib { 
namespace pointcloud {

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief: 基于ceres求解的线面特征ICP
 */
template<typename _PointType>
class CeresEdgeSurfFeatureRegistration : public RegistrationBase<_PointType> {
private:
    using Base = RegistrationBase<_PointType>; 
    using RegistrationResult = typename Base::RegistrationResult;  
public:
    CeresEdgeSurfFeatureRegistration(std::string const& edge_label, std::string const& surf_label);
    void SetInputTarget(FeaturePointCloudContainer<_PointType> const& target_input) override;
    void SetInputTarget(typename Base::MapPtr const& target_input) override;
    void SetInputSource(FeaturePointCloudContainer<_PointType> const& source_input) override;
    void SetMaxIteration(uint16_t const& n) override;
    void SetNormIteration(uint16_t const& n) override;    
    bool Solve(Eigen::Isometry3d &T) override;
    RegistrationResult const& GetRegistrationResult() const override;
    std::vector<std::string> GetUsedPointsName() override;
protected:
    void addSurfCostFactor(ceres::Problem& problem, ceres::LossFunction *loss_function, 
            bool const& save_match_info);
    void addEdgeCostFactor(ceres::Problem& problem, ceres::LossFunction *loss_function,
            bool const& save_match_info);
    void pointAssociateToMap(_PointType const *const pi, _PointType *const po);
private:
    std::string edge_label_, surf_label_;   // 线、面特征的标识
    // target pointcloud 
    typename pcl::PointCloud<_PointType>::ConstPtr surf_point_in_;
    typename pcl::PointCloud<_PointType>::ConstPtr edge_point_in_;
    // 匹配器 
    EdgeFeatureMatch<_PointType> edge_match_;
    SurfFeatureMatch<_PointType> surf_match_;
    // 匹配结果 
    RegistrationResult points_registration_res_;   
    double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
    Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
    Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);
    uint16_t optimization_count_; 
}; // class LineSurfFeatureRegistration 
} // namespace 
}
