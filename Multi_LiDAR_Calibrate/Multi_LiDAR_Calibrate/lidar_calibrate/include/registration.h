// registration.h
#ifndef REGISTRATION_H
#define REGISTRATION_H

#include "feature_extraction.h"   //非常有意思的一个定义。
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <string>
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <pcl/point_types.h> // Include the PCL point types header
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <vector>
#include <string>
#include <cmath> // For std::acos and std::abs
#include <ceres/rotation.h>
#include <Eigen/Geometry> // 包含Eigen几何模块
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>

// 前置声明
struct PlaneData;


// 声明匹配平面方程和点云数据的函数
std::vector<std::pair<PlaneData, PlaneData>> matchPlanes(
        const std::vector<PlaneData>& planes_source,
        const std::vector<PlaneData>& planes_target);


//struct PointCloudAlignmentError {
//    PointCloudAlignmentError(Eigen::Vector3f source_point, Eigen::Vector3f target_point);
//
//    template <typename T>
//    bool operator()(const T* const quaternion, const T* const translation, T* residuals) const;
//
//    static ceres::CostFunction* Create(const Eigen::Vector3f source_point, const Eigen::Vector3f target_point);
//
//    Eigen::Vector3f source_point, target_point;
//};


// 声明优化变换的函数
void optimizeTransformation(const std::vector<std::pair<PlaneData, PlaneData>>& matched_planes);

void optimizeTransformationWithCeres(const std::vector<std::pair<PlaneData, PlaneData>>& matched_planes);



#endif // REGISTRATION_H
