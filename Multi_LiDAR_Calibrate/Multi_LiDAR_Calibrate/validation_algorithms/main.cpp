#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h> // 包含PCL可视化的头文件
#include <thread>  // 对于 std::this_thread::sleep_for
#include <chrono>  // 对于 std::chrono::milliseconds
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>  // 包含变换点云的函数

using namespace pcl;
using namespace std;

int main() {
    // 分别为每个PCD文件创建一个点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_128(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZ>);

    // 指定每个点云文件的路径
    std::string filePath_128 = "/home/fairlee/CLionProjects/lidar_calibrate_validation/validation_algorithms/128.pcd";
    std::string filePath_left = "/home/fairlee/CLionProjects/lidar_calibrate_validation/validation_algorithms/left.pcd";
    std::string filePath_right = "/home/fairlee/CLionProjects/lidar_calibrate_validation/validation_algorithms/right.pcd";

    // 加载每个点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filePath_128, *cloud_128) == -1) {
        PCL_ERROR("Couldn't read file 128_validation_data.pcd\n");
        return -1;
    }
    std::cout << "Loaded " << cloud_128->width * cloud_128->height
              << " data points from 128_validation_data.pcd" << std::endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filePath_left, *cloud_left) == -1) {
        PCL_ERROR("Couldn't read file left_validation_data.pcd\n");
        return -1;
    }
    std::cout << "Loaded " << cloud_left->width * cloud_left->height
              << " data points from left_validation_data.pcd" << std::endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filePath_right, *cloud_right) == -1) {
        PCL_ERROR("Couldn't read file right_validation_data.pcd\n");
        return -1;
    }
    std::cout << "Loaded " << cloud_right->width * cloud_right->height
              << " data points from right_validation_data.pcd" << std::endl;




//// 创建一个PCLVisualizer对象
//    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
//    // 设置背景为黑色
//    viewer.setBackgroundColor(0, 0, 0);
//
//    // 为cloud_128设置红色的颜色处理器
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud_128, 255, 0, 0);
//    // 将cloud_128添加到viewer中，并指定颜色处理器
//    viewer.addPointCloud<pcl::PointXYZ>(cloud_128, red, "cloud 128");
//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud 128");
//
//    // 为cloud_left设置蓝色的颜色处理器
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(cloud_left, 0, 0, 255);
//    // 将cloud_left添加到viewer中，并指定颜色处理器
//    viewer.addPointCloud<pcl::PointXYZ>(cloud_left, blue, "cloud left");
//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud left");
//
//    // 为cloud_right设置紫色的颜色处理器
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> purple(cloud_right, 0, 255, 0);
//    // 将cloud_right添加到viewer中，并指定颜色处理器
//    viewer.addPointCloud<pcl::PointXYZ>(cloud_right, purple, "cloud right");
//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud right");
//
//    // 循环直到'q'被按下关闭窗口
//    while (!viewer.wasStopped()) {
//        viewer.spinOnce(100);
//        std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    }







    // 标定后的点云


   // 创建一个变换后的点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud_left(new pcl::PointCloud<pcl::PointXYZ>);
    // 定义四元数和平移向量
    Eigen::Quaternionf quaternion_left(0.999672,0.0043586, -0.0223607, -0.0116654);
    Eigen::Vector3f translation_left(-0.0752243, 0.240135, 0.244966);
    // 创建一个Affine变换矩阵
    Eigen::Affine3f transform_left = Eigen::Affine3f::Identity();
    // 设置平移部分
    transform_left.translation() << translation_left;
    // 设置旋转部分
    transform_left.rotate(quaternion_left);
    // 使用变换矩阵对点云进行变换
    pcl::transformPointCloud(*cloud_left, *transform_cloud_left, transform_left);
    // 输出变换后的点云，这里仅作示例，根据需要进行保存或处理
    std::cout << "Transformed point cloud has: " << transform_cloud_left->points.size() << " points." << std::endl;







// 创建一个变换后的点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud_right(new pcl::PointCloud<pcl::PointXYZ>);

// 定义四元数和平移向量，使用优化后的值
    Eigen::Quaternionf quaternion_right(0.999029, 0.024304, -0.0331225, 0.0159061);
    Eigen::Vector3f translation_right(-0.0506067, -0.258438, 0.285536);

// 创建一个Affine变换矩阵
    Eigen::Affine3f transform_right = Eigen::Affine3f::Identity();

// 设置平移部分
    transform_right.translation() << translation_right;

// 设置旋转部分
    transform_right.rotate(quaternion_right);

// 使用变换矩阵对点云进行变换
    pcl::transformPointCloud(*cloud_right, *transform_cloud_right, transform_right);

// 输出变换后的点云，这里仅作示例，根据需要进行保存或处理
    std::cout << "Transformed right point cloud has: " << transform_cloud_right->points.size() << " points." << std::endl;



// 创建一个PCLVisualizer对象
    pcl::visualization::PCLVisualizer::Ptr t_viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
// 设置背景为黑色
    t_viewer->setBackgroundColor(0, 0, 0);

// 为cloud_128设置红色的颜色处理器
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud_128, 255, 0, 0);
// 将cloud_128添加到t_viewer中，并指定颜色处理器
    t_viewer->addPointCloud<pcl::PointXYZ>(cloud_128, red, "cloud 128");
    t_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud 128");

//// 为transform_cloud_left设置蓝色的颜色处理器
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(transform_cloud_left, 0, 0, 255);
// 将transform_cloud_left添加到t_viewer中，并指定颜色处理器
    t_viewer->addPointCloud<pcl::PointXYZ>(transform_cloud_left, blue, "transform cloud left");
    t_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transform cloud left");

//// 为transform_cloud_right设置绿色的颜色处理器
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(transform_cloud_right, 0, 255, 0);
// 将transform_cloud_right添加到t_viewer中，并指定颜色处理器
    t_viewer->addPointCloud<pcl::PointXYZ>(transform_cloud_right, green, "transform cloud right");
    t_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transform cloud right");




    // 循环直到'q'被按下关闭窗口
    while (!t_viewer->wasStopped()) {
        t_viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


































    return 0;
}
