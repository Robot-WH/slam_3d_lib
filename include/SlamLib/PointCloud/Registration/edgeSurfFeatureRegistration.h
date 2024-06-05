#pragma once 
#include <execution>  // C++ 17 并行算法 
#include <mutex>
#include <atomic>
#include "registration_base.h"
#include "SlamLib/Ceres/ceres_factor/edge_factor.hpp"
#include "SlamLib/Ceres/ceres_factor/surf_factor.hpp"
#include "SlamLib/Ceres/Parameterization/PoseSE3Parameterization.hpp"
#include "FeatureMatch/EdgeFeatureMatch.hpp"
#include "FeatureMatch/surfFeatureMatch.hpp"
#include "SlamLib/Common/color.hpp"
#include "SlamLib/tic_toc.hpp"
namespace SlamLib { 
namespace pointcloud {
/**
 * @brief: 基于OptimizeMethod::GN/OptimizeMethod::LM法的边缘/面特征匹配  
 */    
template<typename _PointType>
class EdgeSurfFeatureRegistration : public RegistrationBase<_PointType> {
private:
    #define handle_degenerate 1
    using SurfCostFactorInfo = typename SurfFeatureMatch<_PointType>::SurfCostFactorInfo;
    using EdgeCostFactorInfo = typename EdgeFeatureMatch<_PointType>::EdgeCostFactorInfo;
    using Base = RegistrationBase<_PointType>; 
    using RegistrationResult = typename Base::RegistrationResult;  
public:
    enum class OptimizeMethod {GN, LM};
    struct OptionLM {
        uint16_t max_iterater_count_ = 30;   // 最大迭代优化次数   
    };
    struct OptionGN {
        uint16_t max_iterater_count_ = 30;   // 最大迭代优化次数   
    };
    struct Option {
        std::string edge_label_;
        std::string surf_label_;
        OptimizeMethod method_; 
        uint16_t max_iterater_count_ = 10;    // 初始优化迭代次数   每一次迭代需要重新找一次最近邻  
        uint16_t norm_iterater_count_ = 3;  // 常规优化次数
        OptionLM lm_option_;    // LM优化算法的设置 
        OptionGN gn_option_;  
    };  

    EdgeSurfFeatureRegistration(Option option);
    void SetInputTarget(FeaturePointCloudContainer<_PointType> const& target_input) override;
    void SetInputTarget(typename Base::MapPtr const& target_input) override;
    void SetInputSource(FeaturePointCloudContainer<_PointType> const& source_input) override;
    void SetMaxIteration(uint16_t const& n) override;
    void SetNormIteration(uint16_t const& n) override;
    std::vector<std::string> GetUsedPointsName() override;
    bool Solve(Eigen::Isometry3d &T) override;
    RegistrationResult const& GetRegistrationResult() const override;
protected:

    void addSurfCostFactor(bool const& save_match_info);
    void addEdgeCostFactor(bool const& save_match_info);
    void makeHessian(Eigen::Matrix<double, 6, 6>& JTJ, Eigen::Matrix<double, 6, 1>& JTR);
    void checkDegenerate(Eigen::Matrix<double, 6, 6> const& hessian);
    bool gnOptimization(OptionGN const& option);
    bool lmOptimization(OptionLM const& option);
    double calcResidual();
    void updateResidualSingleThread(int const& index_start, int const& index_end, double* res);
    double updateResidual();
    void updateState(Eigen::Matrix<double, 6, 1> const& X);
    void pointLocalToMap(_PointType const *const p_l, _PointType *const p_w);
private:
    Option option_;  
    std::vector<Eigen::Vector3d> origin_surf_points_;
    std::vector<Eigen::Vector3d> origin_edge_points_;
    std::vector<SurfCostFactorInfo> surf_matched_info_;
    std::vector<EdgeCostFactorInfo> edge_matched_info_;
    std::vector<uint32_t> surf_index_;
    std::vector<uint32_t> edge_index_;
    // 匹配器 
    EdgeFeatureMatch<_PointType> edge_match_;
    SurfFeatureMatch<_PointType> surf_match_;
    // target pointcloud 
    typename pcl::PointCloud<_PointType>::ConstPtr surf_point_in_ = nullptr;
    typename pcl::PointCloud<_PointType>::ConstPtr edge_point_in_ = nullptr;
    // 求解结果
    RegistrationResult points_registration_res_;   
    Eigen::Quaterniond q_w_l_;
    Eigen::Vector3d t_w_l_;
    Eigen::MatrixXd Map_;     //  退化时，对于X的remapping 矩阵

    uint16_t optimization_count_ = 0;
    std::atomic<uint16_t> edge_num_{0};
    std::atomic<uint16_t> surf_num_{0};

    bool is_degenerate_ = false; 
    bool check_degenerate_ = false; 
}; // class LineSurfFeatureRegistration 
}
}

