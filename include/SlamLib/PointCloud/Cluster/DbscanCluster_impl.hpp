
#pragma once 
#include "SlamLib/PointCloud/Cluster/DbscanCluster.h"

namespace SlamLib {
namespace pointcloud {

template <typename _PointT>
DBSCANCluster<_PointT>::DBSCANCluster(Option const& option) : option_(option) {}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief: 提取聚类
 * @param cloud_in
 * @param cluster_indices 有效聚类的序号
 * @param outlier_indices 无效聚类(很小)的序号 
 * @return {*}
 */        
template <typename _PointT>
void DBSCANCluster<_PointT>::Extract(
        PCLConstPtr<_PointT> const& cloud_in, std::vector< std::vector<uint32_t>>& cluster_indices, 
        std::vector< std::vector<uint32_t>>& outlier_indices) {
    cluster_indices.clear();
    outlier_indices.clear();  
    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    std::vector<bool> is_outliers(cloud_in->points.size(), false);
    std::vector<PointType> types(cloud_in->points.size(), PointType::UN_PROCESSED);
    std::vector<uint32_t> seed_queue;
    seed_queue.reserve(cloud_in->points.size()); 
    // 构造kdtree
    search_tree_.setInputCloud(cloud_in);
    float search_range = 0; 
    
    for (int i = 0; i < cloud_in->points.size(); i++) {
        if (types[i] == PointType::PROCESSED) {
            continue;
        }
        // 设置搜索范围  距离激光雷达越近则采样越密集  那么搜索距离越小
        // 最小不能小与min_search_range，这个与降采样有关
        search_range = cloud_in->points[i].range * option_.search_range_coeff_; 
        if (search_range > option_.max_search_range) {
            search_range = option_.max_search_range;
        }
        // if (search_range < option_.min_search_range) {
        //     search_range = option_.min_search_range;
        // }
        // 查找该点周围距离在search_range内的其它点
        int nn_size = search_tree_.radiusSearch(cloud_in->points[i], 
            search_range, nn_indices, nn_distances); 

        if (nn_size < option_.min_density_) {
            // 稀疏点提前进行标记  这样以后遇见该点就不用进行搜索了
            is_outliers[i] = true;
            continue;
        }

        // 密度够大  认为是一个聚类 
        // 第一个种子
        seed_queue.push_back(i);
        types[i] = PointType::PROCESSED;
        // 添加该种子周围的邻近点 
        for (int j = 0; j < nn_size; j++) {
            if (types[nn_indices[j]] == PointType::PROCESSED) continue;   
            seed_queue.push_back(nn_indices[j]);    
            types[nn_indices[j]] = PointType::PROCESSED;
        } 
        // 区域生长  
        uint32_t sq_idx = 1;
        while (sq_idx < seed_queue.size()) {
            int cloud_index = seed_queue[sq_idx];
            if (is_outliers[cloud_index]) {
                sq_idx++;
                continue; // no need to check neighbors.
            }
            // 设置搜索范围  距离激光雷达越近则采样越密集  那么搜索距离越小
            // 最小不能小与min_search_range，这个与降采样有关
            search_range = cloud_in->points[cloud_index].range * option_.search_range_coeff_; 
            if (search_range > option_.max_search_range) {
                search_range = option_.max_search_range;
            }
            // if (search_range < option_.min_search_range) {
            //     search_range = option_.min_search_range;
            // }

            nn_size = search_tree_.radiusSearch(cloud_in->points[cloud_index], 
                search_range, nn_indices, nn_distances); 
            // 如果附近点的密度很高，说明该点处与聚类内部，将该点周边点添加为种子 
            if (nn_size >= option_.min_density_) {
                for (int j = 0; j < nn_size; j++) {
                    if (types[nn_indices[j]] == PointType::UN_PROCESSED) {
                        seed_queue.push_back(nn_indices[j]);
                        types[nn_indices[j]] = PointType::PROCESSED;
                    }
                }
            }
            sq_idx++;
        }

        // 聚类完成    
        if (seed_queue.size() >= option_.min_pts_per_cluster_) {       
            cluster_indices.push_back(std::move(seed_queue)); 
        } else {    
            outlier_indices.push_back(std::move(seed_queue)); 
        }
        seed_queue.clear();  
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename _PointT>
void DBSCANCluster<_PointT>::setMinClusterSize (int min_cluster_size) { 
    option_.min_pts_per_cluster_ = min_cluster_size; 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename _PointT>
void DBSCANCluster<_PointT>::setCorePointMinPts(int core_point_min_pts) {
    option_.min_density_ = core_point_min_pts;
}

}
}