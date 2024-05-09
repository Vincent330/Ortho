#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <cmath>

// 相机内外参和平面方程
//float WIDTH = 8192;
//float HEIGHT = 5460;
//float fx = 8141.6649310000003;
//float fy = 8141.6649310000003;
//float cx = 4088.7069849999998;
//float cy = 2744.4657299999999;
//cv::Vec3f plane_normal(0.008542, -0.005626, 1.0);
//cv::Vec3f plane_point(0, 0, -79.328884);
int height = 8192;
int width = 5460;
float fx = 8352.1833208403386;
float fy = 8349.8891564001406;
float cx = 4088.7069849999998;
float cy = 2744.4657299999999;
float step = 0.25;
cv::Vec3f plane_normal(-0.00144578, -0.00226666, 0.999996);
//cv::Vec3f plane_point(0, 0, -91.6242);
cv::Vec3f plane_point(0, 0, -81.6242);

// 存储点的横坐标和纵坐标
class Point2d {
public:
    double x;
    double y;

    Point2d(double x, double y) : x(x), y(y) {}

    static Point2d max;
    static Point2d min;
};
Point2d Point2d::max = Point2d(-1e10, -1e10);
Point2d Point2d::min = Point2d(1e10, 1e10);

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

// 正射图处理
cv::Mat process_image(cv::Mat& image, cv::Mat& new_image, float q_w, float q_x, float q_y, float q_z, float t_x, float t_y, float t_z) {
    
    // 构造相机旋转矩阵
    cv::Matx33f rotation_matrix = quaternion_to_rotation_matrix(q_w, q_x, q_y, q_z);

    // 沿 x 和 y 方向均匀采样三维点并投影到图像上
    for (float y = -100; y < 2000; y += step) {
        for (float x = -100; x < 2000; x += step) {
            float z = (-plane_normal[0] * x - plane_normal[1] * y + plane_point[2]) / plane_normal[2];
            cv::Vec3f point_world(x, y, z);

            // 三维点找像素点
            cv::Vec3f point_camera = -rotation_matrix.t() * point_world + rotation_matrix.t() * cv::Vec3f(t_x, t_y, t_z);
            int v = static_cast<int>((fy * point_camera[1] / point_camera[2] + cy));
            int u = static_cast<int>((fx * point_camera[0] / point_camera[2] + cx));

            int v1 = (y + 100) / step;
            int u1 = (x + 100) / step;

            // 填充像素值到新图像
            if (u >= 0 && u < image.cols && v >= 0 && v < image.rows) {
                //if (new_image.at<cv::Vec3b>(v1, u1) == cv::Vec3b(0, 0, 0)) {
                //    new_image.at<cv::Vec3b>(v1, u1) = image.at<cv::Vec3b>(v, u);
                //}
                new_image.at<cv::Vec3b>(v1, u1) = image.at<cv::Vec3b>(v, u);
            }
        }
    }
    return new_image;
}

int main() {

    //轨迹文件路径
    std::ifstream infile("C:/Users/Jialei He/Desktop/try/enu_tras.txt");
    //std::ifstream infile("C:/Users/Jialei He/Desktop/enu/tras-3.txt");
    if (!infile.is_open()) {
        std::cerr << "Failed to open data file." << std::endl;
        return -1;
    }
    cv::Mat result(2000, 2000, CV_8UC3, cv::Scalar(0, 0, 0));

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
        //图像路径
        std::string filepath = "D:/OpenSfM_1/OpenSfM/data/mydata/images/" + filename;
        cv::Mat image = cv::imread(filepath);
        std::cout << "读取图像 " << filename << std::endl;
        if (image.empty()) {
            std::cerr << "Failed to read image." << std::endl;
            return -1;
        }

        if (first_line) {
            result = process_image(image, result, q_w, q_x, q_y, q_z, t_x, t_y, t_z);
            /*
            Point2d::max.x = t_x > Point2d::max.x ? t_x : Point2d::max.x;
            Point2d::max.y = t_y > Point2d::max.y ? t_y : Point2d::max.y;
            Point2d::min.x = t_x < Point2d::min.x ? t_x : Point2d::min.x;
            Point2d::min.y = t_y < Point2d::min.y ? t_y : Point2d::min.y;
            double width = Point2d::max.x - Point2d::min.x;
            double height = Point2d::max.y - Point2d::min.y;
            int output_width = static_cast<int>(width / step + 100);
            int output_height = static_cast<int>(height / step + 100);
            cv::resize(result, result, cv::Size(output_width, output_height));
            */
            first_line = false;
        }
        else {
            result = process_image(image, result, q_w, q_x, q_y, q_z, t_x, t_y, t_z);
        }
        cv::imwrite("./result.jpg", result);
    }
    return 0;
}
