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
#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>
#include "registration_base.h"

namespace SlamLib { 
namespace pointcloud {

template<typename _PointType>
using ndtomp = pclomp::NormalDistributionsTransform<_PointType, _PointType>; 
template<typename _PointType>
using ndtomp_ptr = std::unique_ptr<ndtomp<_PointType>>;  

/**
 * @brief: 构造 多线程NDT匹配 
 * @details: pclomp
 * @param ndt_resolution ndt网格分辨率
 * @param transformation_epsilon 设置NDT迭代收敛阈值，即迭代增量的大小，
 *                                                                      相当于牛顿法中的delta_x，阈值越小，迭代次数越多；
 * @param step_size Ndt使用了More-Thuente线性搜索求解位姿增量，这里设置搜索的步长
 * @param maximum_iterations 最大优化迭代数量 
 * @param num_threads 线程数量
 * @param nn_search_method 近邻搜索算法 
 */
template<typename _PointType>
static ndtomp_ptr<_PointType> CreateNDTOMP(float const& ndt_resolution,
                                                            float const& transformation_epsilon, 
                                                            float const& step_size,
                                                            int const& maximum_iterations, 
                                                            int const& num_threads,
                                                            std::string const& nn_search_method) {
    // TODO 检查 maximum_iterations， num_threads 是否大于 0  
    ndtomp_ptr<_PointType> ndt_omp(new ndtomp<_PointType>());
    if(num_threads > 0) {
      ndt_omp->setNumThreads(num_threads);      // 3
    }
    ndt_omp->setTransformationEpsilon(transformation_epsilon);    // 步长
    ndt_omp->setMaximumIterations(maximum_iterations);
    ndt_omp->setResolution(ndt_resolution);
    ndt_omp->setStepSize(step_size); 
    // 匹配方法
    if(nn_search_method == "KDTREE") {
      ndt_omp->setNeighborhoodSearchMethod(pclomp::KDTREE);
    } else if (nn_search_method == "DIRECT1") {
      ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT1);
    } else {
      ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    } 
    return std::move(ndt_omp); 
}

/**
 * @brief PCL库 的 点云匹配算法的包装    
 * 
 * @tparam _PointType 
 */
template<typename _PointType>
class PCLRegistration : public RegistrationBase<_PointType> {
public:
    using  RegistrationPtr = std::unique_ptr<pcl::Registration<_PointType, _PointType>>; 

    /**
     * @brief Construct a new PCLRegistration object
     * 
     * @param ptr 具体的匹配算法对象
     * @param used_point_label 当前匹配算法使用的点云标识名称 
     *                                                            (点云特征可能有很多，每个点云匹配算法可能使用不同的特征)
     */
    PCLRegistration(RegistrationPtr ptr, std::string used_point_label);

    /**
     * @brief Set the Input Source object   
     *                这里和PCL的设定刚好相反，这里Source表示匹配的点云，Target表示固定的被匹配的点云
     * FeaturePointCloudContainer格式点云的重载   
     * @param source_input 
     */
    virtual void SetInputSource(FeaturePointCloudContainer<_PointType> 
                                                    const& source_input) override;
    
    /**
     * @brief Set the Input Source object
     *    重载   
     * @param source_input 
     */
    virtual void SetInputSource(
        std::pair<std::string, PCLConstPtr<_PointType>> const& source_input) override;

    /**
     * @brief Set the Input Target object
     *                这里和PCL的设定刚好相反，这里Source表示匹配的点云，Target表示固定的被匹配的点云
     * @param target_input 
     */
    virtual void SetInputTarget(FeaturePointCloudContainer<_PointType> 
                                                    const& target_input) override;

    /**
     * @brief Set the Input Target object
     *                匹配目标为Map类型的重载  
     * @param target_input 
     */
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
