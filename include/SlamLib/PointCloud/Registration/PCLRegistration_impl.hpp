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
#include "SlamLib/PointCloud/Registration/PCLRegistration.h"

namespace SlamLib { 
namespace pointcloud {


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Construct a new Ndt Omp Registration object
 * @param  ptr  pcl::Registration 的子类匹配算法指针  
 * @param  used_point_label 本匹配算法使用的点云标识名
 */
template<typename _PointType>
PCLRegistration<_PointType>::PCLRegistration(RegistrationPtr ptr, 
        std::string used_point_label) : registration_ptr_(std::move(ptr)), 
        used_point_label_(used_point_label) {
    this->used_name_.push_back(used_point_label_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Set the Input Source object
 *                  这里source和pcl的概念相反，即PCL的target点云 
 * @param  source_input  输入数据
 */
template<typename _PointType>
void PCLRegistration<_PointType>::SetInputSource(
        FeaturePointCloudContainer<_PointType> const& source_input) {
    // 因为输入的点云数据很多，因此我们需要找到本匹配算法所需要的点云数据
    if (source_input.find(used_point_label_) != source_input.end()) {
        registration_ptr_->setInputTarget(source_input.at(used_point_label_));
    }
}  

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Set the Input Source object  重载 
 *                  这里source和pcl的概念相反
 * @param  source_input  输入数据
 */
template<typename _PointType>
void PCLRegistration<_PointType>::SetInputSource(
        std::pair<std::string, PCLConstPtr<_PointType>> const& source_input) {
    if (used_point_label_ == source_input.first) {
        registration_ptr_->setInputTarget(source_input.second);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Set the Input Target object，即PCL的source  
 * @param  target_input     输入数据
 */
template<typename _PointType>
void PCLRegistration<_PointType>::SetInputTarget(
        FeaturePointCloudContainer<_PointType> const& target_input) {
    if (target_input.find(used_point_label_) != target_input.end()) {
        registration_ptr_->setInputSource(target_input.at(used_point_label_));
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Set the Input Target object，即PCL的source  
 * @param  target_input     输入数据
 */
template<typename _PointType>
void PCLRegistration<_PointType>::SetInputTarget(
        const typename RegistrationBase<_PointType>::MapPtr& target_input) {
    // 搜索指定名字的地图数据
    const auto& local_map_ptr = target_input->GetLocalMap(used_point_label_);  

    if (local_map_ptr != nullptr) {
        registration_ptr_->setInputSource(local_map_ptr);
    }
}

template<typename _PointType>
void PCLRegistration<_PointType>::SetMaxIteration(uint16_t const& n) {
}

template<typename _PointType>
void PCLRegistration<_PointType>::SetNormIteration(uint16_t const& n) {
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief 求解匹配
 * @param  T source -> target的变换矩阵
 * @return true 
 * @return false 
 */
template<typename _PointType>
bool PCLRegistration<_PointType>::Solve(Eigen::Isometry3d& T) {
    typename pcl::PointCloud<_PointType>::Ptr aligned(new pcl::PointCloud<_PointType>());
    registration_ptr_->align(*aligned, T.matrix().cast<float>());        // 进行配准     predict_trans = setInputSource -> setInputTarget
    //std::cout<<"GetFitnessScore :"<<GetFitnessScore()<<std::endl;
    if (!HasConverged()) {
        return false;
    }

    const Eigen::Matrix4f& res = registration_ptr_->getFinalTransformation();  
    T.translation() =res.cast<double>().block<3, 1>(0, 3);  
    T.linear() = res.cast<double>().block<3, 3>(0, 0); 
    return true;  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename _PointType>
bool PCLRegistration<_PointType>::HasConverged() {
    return registration_ptr_->hasConverged();  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename _PointType>
typename RegistrationBase<_PointType>::RegistrationResult const& 
PCLRegistration<_PointType>::GetRegistrationResult() const {
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename _PointType>
std::vector<std::string> PCLRegistration<_PointType>::GetUsedPointsName() {
    return this->used_name_;  
}  
} // namespace
}
