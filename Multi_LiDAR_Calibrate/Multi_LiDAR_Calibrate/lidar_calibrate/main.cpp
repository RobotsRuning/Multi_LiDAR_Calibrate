#include "feature_extraction.h"
#include "registration.h"
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ceres/ceres.h>



using namespace pcl;
using namespace std;
namespace fs = boost::filesystem;

int main() {

    try {
        string baseDir = "/home/fairlee/lidar_calibrate_data/";
        auto planes_128 = extractPlanesFromCloud(baseDir + "128.pcd");
        auto planes_left = extractPlanesFromCloud(baseDir + "left.pcd");
        auto planes_right = extractPlanesFromCloud(baseDir + "right.pcd");

        //        visualizeAndPrintPlanes(planes_128, "cloud_128");
        //        visualizeAndPrintPlanes(planes_left, "cloud_left");
        //        visualizeAndPrintPlanes(planes_right, "cloud_right");
        auto matched_planes_left = matchPlanes(planes_128, planes_left );
        optimizeTransformationWithCeres(matched_planes_left);



        auto  matched_planes_right = matchPlanes(planes_128, planes_right );
        optimizeTransformationWithCeres(matched_planes_right);




//        // 输出匹配的平面对信息
//        for (const auto& match : matched_planes) {
//            // 输出平面方程参数
//            std::cout << "Matched plane parameters: " << std::endl
//                      << match.first.coefficients->values[0] << ", "
//                      << match.first.coefficients->values[1] << ", "
//                      << match.first.coefficients->values[2] << ", "
//                      << match.first.coefficients->values[3] << " and " << std::endl
//                      << match.second.coefficients->values[0] << ", "
//                      << match.second.coefficients->values[1] << ", "
//                      << match.second.coefficients->values[2] << ", "
//                      << match.second.coefficients->values[3] << std::endl;
//
//            // Additionally, print the number of points in each point cloud
//            std::cout << "Number of points in matched planes: "
//                      << match.first.cloud->points.size() << " and "
//                      << match.second.cloud->points.size() << std::endl;
//        }
//



//// 输出匹配的平面对信息
//        for (const auto& match : matched_planes) {
//            std::cout << "Matched plane parameters: " << std::endl
//                      << "Source: " << match.first.coefficients->values[0] << ", "
//                      << match.first.coefficients->values[1] << ", "
//                      << match.first.coefficients->values[2] << ", "
//                      << match.first.coefficients->values[3] << std::endl
//                      << "Target: " << match.second.coefficients->values[0] << ", "
//                      << match.second.coefficients->values[1] << ", "
//                      << match.second.coefficients->values[2] << ", "
//                      << match.second.coefficients->values[3] << std::endl
//                      << "Number of points: Source = " << match.first.cloud->points.size()
//                      << ", Target = " << match.second.cloud->points.size() << std::endl;
//        }
//











    } catch (const std::exception& e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
