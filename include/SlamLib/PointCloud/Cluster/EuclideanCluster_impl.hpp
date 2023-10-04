
#pragma once 
#include "SlamLib/PointCloud/Cluster/EuclideanCluster.h"

namespace SlamLib {
namespace pointcloud {

template <typename _PointT>
EuclideanCluster<_PointT>::EuclideanCluster(Option const& option) : option_(option) {}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename _PointT>
void EuclideanCluster<_PointT>::Extract(PCLConstPtr<_PointT> const& cloud_in, 
            std::vector< std::vector<uint32_t>>& cluster_indices, 
            std::vector< std::vector<uint32_t>>& outlier_indices) {}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief: 区域生长   
 * @param cloud_in
 * @param seed_cloud 生长种子  
 * @param cluster_indices 有效聚类的序号
 * @return {*}
 */        
template <typename _PointT>
void EuclideanCluster<_PointT>::RegionGrowing(
        PCLConstPtr<_PointT> const& cloud_in, 
        const std::vector<uint32_t>& seed_cloud,
        std::vector< std::vector<uint32_t>>& cluster_indices) {
    std::vector<bool> cluster_flags(cloud_in->points.size(), false);
    // 构造kdtree
    search_tree_.setInputCloud(cloud_in);

    for (const auto& index : seed_cloud) {
        // 如果这个点已经参与聚类了，那么跳过  
        if (cluster_flags[index]) {
            continue;  
        }
        std::vector<uint32_t> seed_queue;
        seed_queue.push_back(index);
        cluster_flags[index] = true;
        int sq_idx = 0;

        while (sq_idx < seed_queue.size()) {
            std::vector<int> nn_indices;
            std::vector<float> nn_distances;
            // 查找该点周围距离在search_range内的其它点
            int nn_size = search_tree_.radiusSearch(cloud_in->points[seed_queue[sq_idx]], 
                0.5, nn_indices, nn_distances); 
            ++sq_idx;  
            if (!nn_size) {
                continue;
            }
            // 将搜索到的邻近点中，之前没有进行过聚类的加入到聚类中
            for (int j = 0; j < nn_size; j++) {
                if (cluster_flags[nn_indices[j]]) continue;   
                seed_queue.push_back(nn_indices[j]);    
                cluster_flags[nn_indices[j]] = true;
            } 
        };

        cluster_indices.push_back(std::move(seed_queue));  
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename _PointT>
void EuclideanCluster<_PointT>::setMinClusterSize (int min_cluster_size) { 
    option_.min_pts_per_cluster_ = min_cluster_size; 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename _PointT>
void EuclideanCluster<_PointT>::setCorePointMinPts(int core_point_min_pts) {
    option_.min_density_ = core_point_min_pts;
}

}
}