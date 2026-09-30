// feature_extraction.cpp
#include "feature_extraction.h"
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <iostream>

// PlaneData构造函数定义
PlaneData::PlaneData() : coefficients(new pcl::ModelCoefficients), cloud(new pcl::PointCloud<pcl::PointXYZ>) {}


// 这个函数尝试从输入的点云中提取一个平面。如果成功，它会返回true。
bool extractPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud, PlaneData &planeData, float distanceThreshold = 0.05) {
    // 创建RANSAC分割器并设置参数
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setOptimizeCoefficients(true); // 让算法优化模型系数
    seg.setModelType(pcl::SACMODEL_PLANE); // 设置分割模型为平面
    seg.setMethodType(pcl::SAC_RANSAC);  // 使用RANSAC作为基础算法
    // seg.setMethodType(pcl::SAC_LMEDS);
    seg.setMaxIterations(2000); // 设置算法的最大迭代次数
    seg.setDistanceThreshold(distanceThreshold);  // 设置点到模型的最大距离阈值，非常重要的一个阈值

    // 进行分割并获取平面模型的系数和内点索引
    seg.setInputCloud(inputCloud); // 设置输入点云
    seg.segment(*inliers, *(planeData.coefficients)); // 进行分割

    // 如果没有找到内点，返回false
    if (inliers->indices.empty()) {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        return false;
    }

    // 使用内点索引提取平面点云
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(inputCloud); // 设置输入点云
    extract.setIndices(inliers); // 设置内点索引
    extract.setNegative(false); // 不反转过滤，即只保留内点
    extract.filter(*(planeData.cloud));  // 提取内点到planeData.cloud

    // 移除提取出的平面点云，留下其余点云以便进一步提取
    extract.setNegative(true); // 反转过滤，即移除内点
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudF(new pcl::PointCloud<pcl::PointXYZ>());
    extract.filter(*cloudF); // 过滤操作
    inputCloud.swap(cloudF); // 更新输入点云为剩余的点云

    return true; // 成功提取平面
}

// 这个函数加载一个点云文件，并尝试从中提取多个平面，最多提取maxPlanes个。
std::vector<PlaneData> extractPlanesFromCloud(const std::string& filePath, int maxPlanes) {
    // 创建新的点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // 尝试从文件中加载点云，如果失败则抛出异常
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filePath, *cloud) == -1) {
        PCL_ERROR("Couldn't read file\n");
        throw std::runtime_error("Failed to load PCD file: " + filePath);
    }
    // 输出加载点云的信息
    std::cout << "Loaded " << cloud->points.size() << " data points from " << filePath << std::endl;

    // 定义PlaneData向量，用于存储提取出的平面数据
    std::vector<PlaneData> planes;
    // 循环提取平面，最多提取maxPlanes个
    for (int i = 0; i < maxPlanes; ++i) {
        PlaneData planeData;
        // 使用extractPlane函数提取平面
        if (extractPlane(cloud, planeData)) {
            // 如果提取成功，将平面数据添加到向量中
            planes.push_back(planeData);
            // 输出提取的平面信息
            // std::cout << "Plane " << i << " has " << planeData.cloud->points.size() << " points." << std::endl;
        } else {
            // 如果没有更多的平面可以提取，中断循环
            std::cout << "No more planes could be extracted." << std::endl;
            break;
        }
    }
    // 返回包含所有提取平面数据的向量
    return planes;
}

void visualizeAndPrintPlanes(const std::vector<PlaneData>& planes, const std::string& cloud_name) {
    // 输出平面参数和点云个数
    std::cout << "Planes in " << cloud_name << ":" << std::endl;
    for (size_t i = 0; i < planes.size(); ++i) {
        std::cout << "Plane " << i << " has " << planes[i].cloud->points.size() << " points." << std::endl;
        std::cout << "Plane " << i << " parameters: "
                  << planes[i].coefficients->values[0] << ", "
                  << planes[i].coefficients->values[1] << ", "
                  << planes[i].coefficients->values[2] << ", "
                  << planes[i].coefficients->values[3] << std::endl;
    }

    // 创建一个可视化对象
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    // 定义一组颜色，每个平面使用不同的颜色
    std::vector<std::vector<float>> colors = {
            {1.0, 0.0, 0.0}, // 红色
            {0.0, 1.0, 0.0}, // 绿色
            {0.0, 0.0, 1.0}, // 蓝色
            // 添加更多颜色以支持更多的平面
    };

    // 遍历平面，为每个平面添加点云到可视化中
    for (size_t i = 0; i < planes.size(); ++i) {
        // 生成一个颜色处理器，为点云着色
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(planes[i].cloud, colors[i % colors.size()][0] * 255, colors[i % colors.size()][1] * 255, colors[i % colors.size()][2] * 255);

        // 添加点云到可视化对象中
        viewer->addPointCloud<pcl::PointXYZ>(planes[i].cloud, single_color, "sample cloud" + std::to_string(i));
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud" + std::to_string(i));
    }

    // 主循环
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}