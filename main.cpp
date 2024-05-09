#include <iostream>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>

// 相机内参
int height = 8192;
int width = 5460;
float fx = 8352.1833208403386;
float fy = 8349.8891564001406;
float cx = 4088.7069849999998;
float cy = 2744.4657299999999;
float step = 0.25;

// 旋转矩阵(相机到世界坐标系的旋转矩阵)
cv::Matx33f quaternion_to_rotation_matrix(float q_w, float q_x, float q_y, float q_z) {
    cv::Matx33f rotation_matrix;
    rotation_matrix(0, 0) = 1 - 2 * q_y * q_y - 2 * q_z * q_z;
    rotation_matrix(0, 1) = 2 * q_x * q_y - 2 * q_w * q_z;
    rotation_matrix(0, 2) = 2 * q_x * q_z + 2 * q_w * q_y;
    rotation_matrix(1, 0) = 2 * q_x * q_y + 2 * q_w * q_z;
    rotation_matrix(1, 1) = 1 - 2 * q_x * q_x - 2 * q_z * q_z;
    rotation_matrix(1, 2) = 2 * q_y * q_z - 2 * q_w * q_x;
    rotation_matrix(2, 0) = 2 * q_x * q_z - 2 * q_w * q_y;
    rotation_matrix(2, 1) = 2 * q_y * q_z + 2 * q_w * q_x;
    rotation_matrix(2, 2) = 1 - 2 * q_x * q_x - 2 * q_y * q_y;
    return rotation_matrix;
}

/*
// 点云统计滤波
void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud) {
    // 创建统计滤波器对象
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input_cloud);
    sor.setMeanK(50); // 设置用于计算平均距离的点的数量
    sor.setStddevMulThresh(1.0); // 设置标准偏差倍数阈值
    sor.filter(*filtered_cloud); // 执行滤波，并存储结果到filtered_cloud中
}

// 点云平面拟合
std::tuple<cv::Vec3f, cv::Vec3f, pcl::PointXYZ, pcl::PointXYZ> fitPlaneFromPointCloud(const std::string& filename) {
    // 加载点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开点云文件\n";
        return std::make_tuple(cv::Vec3f(), cv::Vec3f(), pcl::PointXYZ(), pcl::PointXYZ());
    }

    float x, y, z;
    while (file >> x >> y >> z) {
        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;
        cloud->push_back(point);
    }
    file.close();

    // 进行点云滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filterPointCloud(cloud, filtered_cloud);

    // 平面拟合
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(filtered_cloud);

    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        std::cerr << "平面拟合失败。\n";
        return std::make_tuple(cv::Vec3f(), cv::Vec3f(), pcl::PointXYZ(), pcl::PointXYZ());
    }

    cv::Vec3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]); // 平面法向量
    cv::Vec3f point(0, 0, -coefficients->values[3] / coefficients->values[2]); // 提取平面上一点

    // 找到点云中的最小和最大坐标值
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*filtered_cloud, minPt, maxPt);

    return std::make_tuple(normal, point, minPt, maxPt);
}
*/

// 点云平面拟合
std::tuple<cv::Vec3f, cv::Vec3f, pcl::PointXYZ, pcl::PointXYZ> fitPlaneFromPointCloud(const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开点云文件\n";
        return std::make_tuple(cv::Vec3f(), cv::Vec3f(), pcl::PointXYZ(), pcl::PointXYZ());
    }

    float x, y, z;
    while (file >> x >> y >> z) {
        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;
        cloud->push_back(point);
    }
    file.close();

    // 平面拟合
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud);

    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        std::cerr << "平面拟合失败。\n";
        return std::make_tuple(cv::Vec3f(), cv::Vec3f(), pcl::PointXYZ(), pcl::PointXYZ());
    }

    // 提取平面法向量
    cv::Vec3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

    // 提取平面上一点
    cv::Vec3f point(0, 0, -coefficients->values[3] / coefficients->values[2]);

    // 找到点云中的最小和最大坐标值
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);

    return std::make_tuple(normal, point, minPt, maxPt);
}

// 正射图处理
cv::Mat process_image(cv::Mat& image, cv::Mat& new_image, float q_w, float q_x, float q_y, float q_z, float t_x, float t_y, float t_z, const cv::Vec3f& plane_normal, const cv::Vec3f& plane_point, const pcl::PointXYZ& minPt, const pcl::PointXYZ& maxPt) {

    // 构造相机旋转矩阵
    cv::Matx33f rotation_matrix = quaternion_to_rotation_matrix(q_w, q_x, q_y, q_z);

    // 沿 x 和 y 方向均匀采样三维点并投影到图像上
    for (float y = minPt.y - 5; y < maxPt.y + 15; y += step) {
        for (float x = minPt.x - 50; x < maxPt.x + 15; x += step) {
            float z = (-plane_normal[0] * x - plane_normal[1] * y + plane_point[2]) / plane_normal[2];
            cv::Vec3f point_world(x, y, z);

            // 三维点找像素点
            cv::Vec3f point_camera = -rotation_matrix.t() * point_world + rotation_matrix.t() * cv::Vec3f(t_x, t_y, t_z);
            int v = static_cast<int>((fy * point_camera[1] / point_camera[2] + cy));
            int u = static_cast<int>((fx * point_camera[0] / point_camera[2] + cx));

            int v1 = (y - minPt.y + 5) / step;
            int u1 = (x - minPt.x + 50) / step;

            // 填充像素值到新图像
            if (u >= 0 && u < image.cols && v >= 0 && v < image.rows) {
                new_image.at<cv::Vec3b>(v1, u1) = image.at<cv::Vec3b>(v, u);
            }
        }
    }
    return new_image;
}

int main() {
    
    // 点云文件路径
    std::string cloudname = "./测试数据及结果/points_w.txt";
    std::tuple<cv::Vec3f, cv::Vec3f, pcl::PointXYZ, pcl::PointXYZ> plane = fitPlaneFromPointCloud(cloudname);
    cv::Vec3f plane_normal = std::get<0>(plane);
    cv::Vec3f plane_point = std::get<1>(plane);
    pcl::PointXYZ minPt = std::get<2>(plane);
    pcl::PointXYZ maxPt = std::get<3>(plane);
    int targetWidth = static_cast<int>(maxPt.x - minPt.x + 65)/step;
    int targetHeight = static_cast<int>(maxPt.y - minPt.y + 20)/step;

    // 轨迹文件路径
    std::ifstream infile("./测试数据及结果/images_w.txt");
    if (!infile.is_open()) {
        std::cerr << "Failed to open data file." << std::endl;
        return -1;
    }
    cv::Mat result(targetHeight, targetWidth, CV_8UC3, cv::Scalar(0, 0, 0));

    std::string line;
    bool first_line = true;

    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        std::string filename;
        float q_w, q_x, q_y, q_z, t_x, t_y, t_z;
        if (!(iss >> filename >> q_w >> q_x >> q_y >> q_z >> t_x >> t_y >> t_z)) {
            std::cerr << "Failed to parse line: " << line << std::endl;
            continue;
        }
        
        // 图像路径
        std::string imagepath = "./测试数据及结果/images/" + filename;
        cv::Mat image = cv::imread(imagepath);
        std::cout << "读取图像 " << filename << std::endl;
        if (image.empty()) {
            std::cerr << "Failed to read image." << std::endl;
            return -1;
        }

        if (first_line) {
            result = process_image(image, result, q_w, q_x, q_y, q_z, t_x, t_y, t_z, plane_normal, plane_point, minPt, maxPt);
            first_line = false;
        }
        else {
            result = process_image(image, result, q_w, q_x, q_y, q_z, t_x, t_y, t_z, plane_normal, plane_point, minPt, maxPt);
        }
        cv::imwrite("./测试数据及结果/result.jpg", result);
    }
    return 0;
}
