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
        // 如果出现了新的点云特征数据 
        if (local_map_frame_container_.find(name) == local_map_frame_container_.end()) {
            this->local_map_container_[name].reset(new pcl::PointCloud<_PointType>());  // 初始化新的点云local map 
            ivox_container_.insert(std::make_pair(name, IVox<_PointType>(option_.ivox_option_)));  // 为该特征点云创建新的ivox
        }
        // 获取当前特征点云的滑动窗口
        auto& frame_queue_ = local_map_frame_container_[name];
        typename pcl::PointCloud<_PointType>::ConstPtr delete_frame;  
        // 更新滑动窗口  
        if (frame_queue_.size() >= option_.window_size_) {  
            this->full_ = true;   
            delete_frame = frame_queue_.front();  
            frame_queue_.pop_front();
            frame_queue_.push_back(frame);
            this->local_map_container_[name]->clear();   
            // 更新submap  只是为了可视化3D local map  
            for (typename std::deque<PCLConstPtr<_PointType>>::const_iterator it = frame_queue_.begin(); 
                it != frame_queue_.end(); it++) {
                    *this->local_map_container_[name] += **it;   
            }
            // ivox删除最老帧
            ivox_container_.at(name).DeletePoints(*delete_frame);
        } else {  
            *this->local_map_container_[name] += *frame;      
            frame_queue_.push_back(frame);   
        }
        // ivox添加最新帧  
        ivox_container_.at(name).AddPoints(*frame);
        // std::cout<<"name size: "
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
        if (local_map_frame_container_.find(name) == local_map_frame_container_.end()) {
            this->local_map_container_[name].reset(new pcl::PointCloud<_PointType>()); 
            ivox_container_.insert(std::make_pair(name, IVox<_PointType>(option_.ivox_option_)));
        }
        auto& frame_queue_ = local_map_frame_container_[name];
        if (!frame_queue_.empty()) {
            typename pcl::PointCloud<_PointType>::ConstPtr delete_frame;  
            delete_frame = frame_queue_.back();  
            frame_queue_.pop_back();
            ivox_container_.at(name).DeletePoints(*delete_frame);
        }
        frame_queue_.push_back(frame);
        ivox_container_.at(name).AddPoints(*frame);
        // 更新submap  
        this->local_map_container_[name]->clear();   
        for (typename std::deque<PCLConstPtr<_PointType>>::const_iterator it = frame_queue_.begin(); 
            it != frame_queue_.end(); it++) {
            *this->local_map_container_[name] += **it;   
        }
        return;  
    }

    /**
     * @brief 将滑动窗口地图中标识为name的滑窗地图的第n个点云数据用frame进行替代 
     *                  
     * @param name 
     * @param n 
     * @param frame 新的数据
     */
    virtual void AmendData(std::string const& name, int const& n,
            typename pcl::PointCloud<_PointType>::ConstPtr frame) override {
        if (frame->empty()) {
            std::cout << "AmendData ---- frame->empty()" << std::endl;
            return; 
        } 
        if (n < 0 || n >= GetWindowSize(name)) {
            std::cout << "AmendData ---- invalid n: " << n << std::endl;
            return; 
        }
        std::lock_guard<std::mutex> lock(this->local_map_mt_);
        if (local_map_frame_container_.find(name) == local_map_frame_container_.end()) {
            std::cout << "AmendData ---- invalid name" << std::endl;
            return;
        }
        auto& frame_queue_ = local_map_frame_container_[name];
        ivox_container_.at(name).DeletePoints(*frame_queue_[n]);
        std::cout << "修改前size: " << frame_queue_[n]->size() << std::endl;
        frame_queue_[n] = frame;
        std::cout << "修改后size: " << frame_queue_[n]->size() << std::endl;
        ivox_container_.at(name).AddPoints(*frame);
    }

    /**
     * @brief 获取当前标识为name的特征的滑窗大小
     * 
     * @param name 
     * @return int 
     */
    virtual int GetWindowSize(std::string const& name) const override {
        if (local_map_frame_container_.find(name) == local_map_frame_container_.end()) {
            return -1;
        }
        return local_map_frame_container_.at(name).size();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /**
     * @brief: 搜索localmap
     * @return 是否搜索到 num 个点 
     */            
    virtual bool GetNearlyNeighbor(std::string const& name, _PointType const& point, 
            uint16_t const& num, double const& max_range, PointVector& res) const override {
        if (ivox_container_.find(name) == ivox_container_.end()) return false;   
        // ivox 中 num 表示最多提取的点的数量，而我们这里标识的是最少提取点的数量   
        //TicToc tt;
        ivox_container_.at(name).GetClosestPoint(point, res, num, max_range); 
        if (res.size() < num) {
            return false; 
        }    
        //tt.toc("ivox knn ");
        return true;  
    } 
    
    void Reset() override {
        PointCloudLocalMapBase<_PointType>::Reset();  
        ivox_container_.clear();
        local_map_frame_container_.clear();  
    }

private:
    Option option_; 
    std::unordered_map<std::string, IVox<_PointType>> ivox_container_;  
    std::unordered_map<std::string, 
        std::deque<typename pcl::PointCloud<_PointType>::ConstPtr>> local_map_frame_container_;  
}; // class IvoxTimedSlidingLocalMap
}
}
