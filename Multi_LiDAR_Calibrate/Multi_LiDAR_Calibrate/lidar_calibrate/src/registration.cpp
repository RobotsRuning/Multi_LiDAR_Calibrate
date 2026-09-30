#include "registration.h"


// Helper function to calculate the angle between two vectors
// 计算两个三维向量之间的夹角（以度为单位）
double angleBetweenVectors(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2) {
    // 计算两个向量的点积。点积的几何解释是：两个向量长度的乘积与它们夹角余弦值的乘积。
    double dot = v1.dot(v2);
    // 计算第一个向量的长度的平方。向量的点积自身等于其长度的平方。
    double lenSq1 = v1.dot(v1);
    // 计算第二个向量的长度的平方。
    double lenSq2 = v2.dot(v2);
    // 计算两个向量之间的夹角。使用acos（反余弦函数）来得到夹角的弧度值。
    // acos的参数是两向量的点积除以它们长度的乘积的平方根，这个值等于两向量夹角的余弦值。
    double angle = acos(dot / sqrt(lenSq1 * lenSq2));
    // 将夹角从弧度转换为度。因为π弧度等于180度，所以乘以(180.0 / M_PI)来进行转换。
    return angle * (180.0 / M_PI); // Convert to degrees
}


/*
 * 这个算法的核心是通过量化平面方程的相似度来匹配平面，
 * 具体通过计算法向量的夹角和平面到原点的距离差异来实现。
 * 这种方法在处理具有相似几何结构的点云数据时非常有用，
 * 例如在点云配准、模型识别或几何分析中。
 * 通过这种方式，
 * 我们可以自动识别并匹配来自不同扫描或不同观测点的相似几何特征，
 * 为进一步的数据处理（如配准、融合或比较）提供基础。
 */

// 匹配两组平面，基于它们的法向量和到原点的距离。
std::vector<std::pair<PlaneData, PlaneData>> matchPlanes(
        const std::vector<PlaneData>& planes_source, // 源平面集合
        const std::vector<PlaneData>& planes_target) { // 目标平面集合
    // 存储最终匹配结果的向量，每个匹配是一个平面对（源平面和目标平面）
    std::vector<std::pair<PlaneData, PlaneData>> matched_planes;
    // 遍历源平面集合中的每一个平面
    for (const auto& source_plane : planes_source) {
        // 初始化最佳匹配分数为无限大，用于寻找最小分数的匹配
        double best_match_score = std::numeric_limits<double>::infinity();
        // 初始化最佳匹配索引，用于记录当前最佳匹配的目标平面索引
        int best_match_index = -1;
        // 提取当前源平面的法向量和D值
        Eigen::Vector3f source_normal(source_plane.coefficients->values[0],
                                      source_plane.coefficients->values[1],
                                      source_plane.coefficients->values[2]);
        double source_d = source_plane.coefficients->values[3];
        // 遍历目标平面集合，寻找与当前源平面最匹配的平面
        for (size_t j = 0; j < planes_target.size(); ++j) {
            const auto& target_plane = planes_target[j];
            // 提取目标平面的法向量和D值
            Eigen::Vector3f target_normal(target_plane.coefficients->values[0],
                                          target_plane.coefficients->values[1],
                                          target_plane.coefficients->values[2]);
            double target_d = target_plane.coefficients->values[3];
            // 计算当前源平面和目标平面之间法向量的夹角差异
            double angle_diff = angleBetweenVectors(source_normal, target_normal);
            // 计算D值的绝对差异
            double d_diff = std::abs(source_d - target_d);
            // 定义匹配分数为夹角差异和D值差异的和，分数越小表示匹配度越高
            double score = angle_diff + d_diff;
            // 如果当前分数是目前为止最佳的，更新最佳匹配分数和索引
            if (score < best_match_score) {
                best_match_score = score;
                best_match_index = j;
            }
        }
        // 如果找到了最佳匹配平面，将该匹配添加到结果向量中
        if (best_match_index != -1) {
            matched_planes.push_back(std::make_pair(source_plane, planes_target[best_match_index]));
        }
    }
    // 返回匹配的平面对集合
    return matched_planes;
}






/*
 * 它基于给定的变换参数（四元数和平移向量）计算每个点变换后到指定平面的距离，
 * 并将这个距离作为优化的目标（残差）。
 * Ceres Solver通过迭代优化这些残差，
 * 寻找最小化所有残差总和的变换参数，从而实现点云对齐到平面的目标。
 */

struct PlaneAlignmentCostFunctor {

    // 构造函数，接收一个3D点和一个平面方程参数
    // point - 表示点云中的一个点
    // plane - 表示平面方程的参数，形式为(ax + by + cz + d = 0)，其中(a, b, c)是平面的法向量，d是到原点的距离

    PlaneAlignmentCostFunctor(const Eigen::Vector3f& point,
                              const Eigen::Vector4f& plane)  //结构体的构造函数声明
            : point_(point), plane_(plane) {}
    // 构造函数的初始化列表，用于直接初始化成员变量point_和plane_。

    // 重载()运算符，使得此结构体可以作为Ceres的代价函数
    // quaternion - 表示旋转的四元数，格式为(x, y, z, w)
    // translation - 表示平移向量，格式为(tx, ty, tz)
    // residual - 用于存储计算出的残差（即距离）
    template <typename T>
    bool operator()(const T* const quaternion, const T* const translation, T* residual) const {

        // 将输入的四元数和平移向量转换为Eigen的四元数和向量类型
        Eigen::Quaternion<T> q(quaternion[3], quaternion[0], quaternion[1], quaternion[2]);
        Eigen::Matrix<T, 3, 1> t(translation[0], translation[1], translation[2]);


        // 使用四元数和平移向量变换点，实现点的旋转和平移
        // point_.cast<T>()将原始点的类型转换为Ceres优化使用的数值类型（例如double或float）

        Eigen::Matrix<T, 3, 1> point_transformed = q * point_.cast<T>() + t;

        // 计算变换后的点到平面的距离
        // plane_.head<3>()提取平面方程的法向量(a, b, c)，plane_(3)获取平面方程的d参数
        // .dot()计算点乘，.norm()计算向量的模（长度）

        T distance = (plane_.head<3>().cast<T>().dot(point_transformed) + T(plane_(3))) /
                     plane_.head<3>().cast<T>().norm();

        // 将计算出的距离作为残差
        residual[0] = distance;

        return true;     // 返回true表示计算成功
    }
private:
    const Eigen::Vector3f point_;    // 存储原始点
    const Eigen::Vector4f plane_;    // 存储平面方程参数
};




// 定义函数，用于通过Ceres Solver优化变换参数
void optimizeTransformationWithCeres(const std::vector<std::pair<PlaneData, PlaneData>>& matched_planes) {

    // 初始化四元数和平移向量，四元数为单位四元数，表示无旋转
    double quaternion[4] = {0.0, 0.0, 0.0, 1.0};
    double translation[3] = {0.0, 0.0, 0.0};

    // 创建Ceres Problem实例
    ceres::Problem problem;
    ceres::LocalParameterization *q_manifold = new ceres::EigenQuaternionParameterization();
    problem.AddParameterBlock(quaternion,4,q_manifold);
    problem.AddParameterBlock(translation,3);

    // 为每对匹配的平面添加所有点到对应平面的距离代价
    for (const auto& match : matched_planes) {

        //用左边的雷达或者右边的雷达的点云到主雷达平面的距离
        // 不要用主雷达的点云到左边的雷达或者右边的平面
        //
        //
        //

        const auto& source_points = match.second.cloud->points;
        const auto& target_plane = match.first.coefficients->values;

        for (const auto& point : source_points) {
            Eigen::Vector3f point_eigen(point.x, point.y, point.z);
            Eigen::Vector4f plane_eigen(target_plane[0], target_plane[1], target_plane[2], target_plane[3]);

            ceres::CostFunction* cost_function =
                    new ceres::AutoDiffCostFunction<PlaneAlignmentCostFunctor, 1, 4, 3>(
                            new PlaneAlignmentCostFunctor(point_eigen, plane_eigen));

            problem.AddResidualBlock(cost_function, nullptr, quaternion, translation);
        }
    }

    // 配置并运行求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << "\n";
    std::cout << "Optimized quaternion: [" << quaternion[0] << ", " << quaternion[1]
              << ", " << quaternion[2] << ", " << quaternion[3] << "]\n";
    std::cout << "Optimized translation: [" << translation[0] << ", "
              << translation[1] << ", " << translation[2] << "]\n";
}

