
#pragma once 
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "SlamLib/Common/pointcloud.h"

namespace SlamLib {
namespace pointcloud {

template <typename _PointT>
class EuclideanCluster {
public:
    using Ptr = std::unique_ptr<EuclideanCluster<_PointT>>;
    struct Option {
        uint16_t min_density_ = 10;   // 密度 ：一定范围内的点的数量
        uint16_t min_pts_per_cluster_ = 3; // 合法聚类的最小点数量 
        float search_range_coeff_ = 0.01; // 转换系数  ， 搜索范围 = search_range_coeff_ * range 
        float max_search_range = 1; // 最大搜索范围
        float min_search_range = 0; // 最小搜索范围
    };
    EuclideanCluster(Option const& option);
    void Extract(PCLConstPtr<_PointT> const& cloud_in, 
            std::vector< std::vector<uint32_t>>& cluster_indices, 
            std::vector< std::vector<uint32_t>>& outlier_indices);
    void RegionGrowing(
        PCLConstPtr<_PointT> const& cloud_in, 
        const std::vector<uint32_t>& seed_cloud,
        std::vector< std::vector<uint32_t>>& cluster_indices);
    void setMinClusterSize (int min_cluster_size);
    void setCorePointMinPts(int core_point_min_pts);

protected:
    enum class PointType {
        UN_PROCESSED = 0,
        PROCESSED
    };
    Option option_;  
    PCLPtr<_PointT> input_cloud_;
    pcl::KdTreeFLANN<_PointT> search_tree_;
}; // class EuclideanCluster
}
}