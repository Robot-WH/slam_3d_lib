
#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>

namespace SlamLib {
namespace utility {

constexpr double CoefDegreeToRadian = M_PI / 180.;
constexpr double CoefRadianToDegree = 180. / M_PI;

// 对角度进行标准化     [-M_PI, M_PI]
static void NormalizeAngle(double& angle) {        
    if(angle >= M_PI)
        angle -= 2.0*M_PI;
    if(angle < -M_PI)
        angle += 2.0*M_PI;
}

/**
 * squared distance
 * @param p1
 * @param p2
 * @return
 */
template<class PointType>
static inline float CalcDist(const PointType &p1, const PointType &p2) {
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
}

static inline float CalcDist(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2) { 
    return (p1 - p2).squaredNorm(); 
}


}
}


