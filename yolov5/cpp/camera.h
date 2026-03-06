#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

/**
 * 相机类：管理相机内外参，提供像素坐标与世界坐标的互相转换
 *
 * CSV 参数文件格式（9行 x 3列）：
 *   第 0-2 行：内参矩阵 (3x3)
 *   第 3   行：[k1, k2, k3]（径向畸变）
 *   第 4   行：[p1, p2, 0]（切向畸变）
 *   第 5-7 行：旋转矩阵 (3x3)
 *   第 8   行：平移向量 [tx, ty, tz]
 */
class Camera
{
public:
  Camera();
  Camera(const std::string &params_file, int width = 1920, int height = 1080);

  bool loadFromCSV(const std::string &params_file);
  bool isCalibrated() const { return is_calibrated_; }

  /**
   * 像素坐标 → 世界坐标（假设目标在已知高度的水平面上）
   * @param img_point  图像像素坐标 (u, v)
   * @param height     目标 Z 轴高度（地面目标传 0.0）
   * @param x          输出：世界坐标 X
   * @param y          输出：世界坐标 Y
   */
  bool imageToWorld(cv::Point2f img_point, double height,
                    double &x, double &y) const;

  /**
   * 世界坐标 → 像素坐标
   * @param world_point  世界坐标 (X, Y, Z)
   * @param img_point    输出：像素坐标 (u, v)
   */
  bool transform(cv::Point3d world_point, cv::Point2i &img_point) const;

  /**
   * 在图像上绘制世界坐标系的三个坐标轴（X红/Y蓝/Z紫）
   */
  cv::Mat drawCoordinateSystem(const cv::Mat &img, double axis_length = 1.0,
                               int line_thickness = 1) const;

private:
  int width_;
  int height_;

  cv::Mat intrinsic_matrix_;        // 3x3 内参矩阵
  cv::Mat distortion_coefficients_; // 1x5 畸变系数 [k1,k2,p1,p2,k3]
  cv::Mat rotation_matrix_;         // 3x3 旋转矩阵
  cv::Mat rotation_vector_;         // 3x1 旋转向量（Rodrigues）
  cv::Mat translation_vector_;      // 3x1 平移向量

  cv::Mat projection_matrix_; // 3x4，预计算的 T = K * [R | t]

  bool is_calibrated_;
};
