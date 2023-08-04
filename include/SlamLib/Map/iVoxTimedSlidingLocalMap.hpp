#pragma once
#include <deque>
#include "LocalMap.hpp"
#include "ivox3d/ivox3d.h"  
namespace SlamLib {
namespace map {
/**
 * @brief:  ivox + 时间滑动窗口 
 * @param _PointType local map中每个地图点的类型  
 */    
template<typename _PointType>
class IvoxTimedSlidingLocalMap : public PointCloudLocalMapBase<_PointType> {
protected:
    using PointVector = typename PointCloudLocalMapBase<_PointType>::PointVector; 
public:
    using IvoxNearbyType = typename IVox<_PointType>::NearbyType;
    struct  Option {
        int window_size_ = 10; 
        typename IVox<_PointType>::Options ivox_option_;  
    };

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    IvoxTimedSlidingLocalMap(Option option) : option_(option) {
        LOG(INFO) << "create IvoxTimedSlidingLocalMap"; 
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 由于发生了充足的运动，因此添加一帧数据到Local map   
     */            
    void UpdateLocalMapForMotion(std::string const& name, 
            typename pcl::PointCloud<_PointType>::ConstPtr const& frame) override {
        if (frame->empty()) return;  
        std::lock_guard<std::mutex> lock(this->local_map_mt_);
        std::string map_name = name + "_localmap";  
        // 如果出现了新的点云特征数据 
        if (local_map_frame_container_.find(map_name) == local_map_frame_container_.end()) {
            this->local_map_container_[map_name].reset(new pcl::PointCloud<_PointType>());  // 初始化新的点云local map 
            ivox_container_.insert(std::make_pair(map_name, IVox<_PointType>(option_.ivox_option_)));  // 为该特征点云创建新的ivox
        }
        // 获取当前特征点云的滑动窗口
        auto& frame_queue_ = local_map_frame_container_[map_name];
        typename pcl::PointCloud<_PointType>::ConstPtr delete_frame;  
        // 更新滑动窗口  
        if (frame_queue_.size() >= option_.window_size_) {  
            this->full_ = true;   
            delete_frame = frame_queue_.front();  
            frame_queue_.pop_front();
            frame_queue_.push_back(frame);
            this->local_map_container_[map_name]->clear();   
            // 更新submap  只是为了可视化3D local map  
            for (typename std::deque<PCLConstPtr<_PointType>>::const_iterator it = frame_queue_.begin(); 
                it != frame_queue_.end(); it++) {
                    *this->local_map_container_[map_name] += **it;   
            }
            // ivox删除最老帧
            ivox_container_.at(map_name).DeletePoints(*delete_frame);
        } else {  
            *this->local_map_container_[map_name] += *frame;      
            frame_queue_.push_back(frame);   
        }
        // ivox添加最新帧  
        ivox_container_.at(map_name).AddPoints(*frame);
        // std::cout<<"map_name size: "
        // <<this->local_map_.second->size()<<std::endl;
        return;  
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 由于长时间没有更新地图，因此添加一帧数据到Local map   
     * @details 此时直接将滑动窗口最末尾的数据移除
     */            
    void UpdateLocalMapForTime(std::string const& name, 
            typename pcl::PointCloud<_PointType>::ConstPtr const& frame) override {
        if (frame->empty()) return;  
        std::lock_guard<std::mutex> lock(this->local_map_mt_);
        std::string map_name = name + "_localmap";  
        if (local_map_frame_container_.find(map_name) == local_map_frame_container_.end()) {
            this->local_map_container_[map_name].reset(new pcl::PointCloud<_PointType>()); 
            ivox_container_.insert(std::make_pair(map_name, IVox<_PointType>(option_.ivox_option_)));
        }
        auto& frame_queue_ = local_map_frame_container_[map_name];
        if (!frame_queue_.empty()) {
            typename pcl::PointCloud<_PointType>::ConstPtr delete_frame;  
            delete_frame = frame_queue_.back();  
            frame_queue_.pop_back();
            ivox_container_.at(map_name).DeletePoints(*delete_frame);
        }
        frame_queue_.push_back(frame);
        ivox_container_.at(map_name).AddPoints(*frame);
        // 更新submap  
        this->local_map_container_[map_name]->clear();   
        for (typename std::deque<PCLConstPtr<_PointType>>::const_iterator it = frame_queue_.begin(); 
            it != frame_queue_.end(); it++) {
            *this->local_map_container_[map_name] += **it;   
        }
        return;  
    }
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 搜索localmap
     * @return 是否搜索到 num 个点 
     */            
    virtual bool GetNearlyNeighbor(std::string const& name, _PointType const& point, 
            uint16_t const& num, double const& max_range, PointVector& res) const override {
        std::string map_name = name + "_localmap";  
        if (ivox_container_.find(map_name) == ivox_container_.end()) return false;   
        // ivox 中 num 表示最多提取的点的数量，而我们这里标识的是最少提取点的数量   
        //TicToc tt;
        ivox_container_.at(map_name).GetClosestPoint(point, res, num, max_range); 
        if (res.size() < num) {
            return false; 
        }    
        //tt.toc("ivox knn ");
        return true;  
    } 

private:
    Option option_; 
    std::unordered_map<std::string, IVox<_PointType>> ivox_container_;  
    std::unordered_map<std::string, 
        std::deque<typename pcl::PointCloud<_PointType>::ConstPtr>> local_map_frame_container_;  
}; // class IvoxTimedSlidingLocalMap
}
}
