// feature_extraction.h
#ifndef FEATURE_EXTRACTION_H
#define FEATURE_EXTRACTION_H

#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <string>
#include <vector>
#include <iostream>
#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include <iostream>


// 声明一个用于存储平面数据的结构体
struct PlaneData {
    pcl::ModelCoefficients::Ptr coefficients; // 平面参数
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud; // 对应的点云
    PlaneData();
};


// 声明提取点云中平面的函数
std::vector<PlaneData> extractPlanesFromCloud(const std::string& filePath, int maxPlanes = 3);
void visualizeAndPrintPlanes(const std::vector<PlaneData>& planes, const std::string& cloud_name);

#endif // FEATURE_EXTRACTION_H

