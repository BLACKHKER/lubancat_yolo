#include "camera.h"

#include <fstream>
#include <sstream>
#include <cmath>
#include <cstdio>

Camera::Camera() : width_(1920), height_(1080), is_calibrated_(false) {}

Camera::Camera(const std::string &params_file, int width, int height)
    : width_(width), height_(height), is_calibrated_(false)
{
  loadFromCSV(params_file);
}

bool Camera::loadFromCSV(const std::string &params_file)
{
  std::ifstream file(params_file);
  if (!file.is_open())
  {
    printf("Camera: 文件未找到: %s\n", params_file.c_str());
    return false;
  }

  std::vector<std::vector<double>> params;
  std::string line;
  while (std::getline(file, line))
  {
    if (line.empty())
      continue;
    std::vector<double> row;
    std::stringstream ss(line);
    std::string val;
    while (std::getline(ss, val, ','))
    {
      try
      {
        row.push_back(std::stod(val));
      }
      catch (...)
      {
      }
    }
    if (!row.empty())
      params.push_back(row);
  }

  if (params.size() < 9)
  {
    printf("Camera: CSV 行数不足（需要至少 9 行，实际 %zu 行）\n", params.size());
    return false;
  }

  // 内参矩阵（第 0-2 行）
  intrinsic_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      intrinsic_matrix_.at<double>(i, j) = params[i][j];

  // 畸变系数 [k1, k2, p1, p2, k3]
  distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
  distortion_coefficients_.at<double>(0) = params[3][0]; // k1
  distortion_coefficients_.at<double>(1) = params[3][1]; // k2
  distortion_coefficients_.at<double>(2) = params[4][0]; // p1
  distortion_coefficients_.at<double>(3) = params[4][1]; // p2
  distortion_coefficients_.at<double>(4) = params[3][2]; // k3

  // 旋转矩阵（第 5-7 行）
  rotation_matrix_ = cv::Mat(3, 3, CV_64F);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      rotation_matrix_.at<double>(i, j) = params[5 + i][j];

  // 平移向量（第 8 行，列向量）
  translation_vector_ = cv::Mat(3, 1, CV_64F);
  for (int i = 0; i < 3; i++)
    translation_vector_.at<double>(i) = params[8][i];

  // 旋转向量（Rodrigues）
  cv::Rodrigues(rotation_matrix_, rotation_vector_);

  // 预计算投影矩阵 T = K * [R | t]（3x4）
  cv::Mat Rt;
  cv::hconcat(rotation_matrix_, translation_vector_, Rt);
  projection_matrix_ = intrinsic_matrix_ * Rt;

  is_calibrated_ = true;
  printf("Camera: 内外参加载成功\n");
  return true;
}

bool Camera::imageToWorld(cv::Point2f img_point, double height,
                          double &x, double &y) const
{
  if (!is_calibrated_)
  {
    printf("Camera: 相机参数未标定\n");
    return false;
  }

  // 1. 畸变矫正
  std::vector<cv::Point2f> pts_in = {img_point};
  std::vector<cv::Point2f> pts_out;
  cv::undistortPoints(pts_in, pts_out,
                      intrinsic_matrix_, distortion_coefficients_,
                      cv::noArray(), intrinsic_matrix_);
  const cv::Point2f &ud = pts_out[0];

  cv::Mat eq = cv::Mat::zeros(3, 3, CV_64F);
  for (int i = 0; i < 3; i++)
  {
    eq.at<double>(i, 0) = projection_matrix_.at<double>(i, 0);
    eq.at<double>(i, 1) = projection_matrix_.at<double>(i, 1);
    eq.at<double>(i, 2) = height * projection_matrix_.at<double>(i, 2) + projection_matrix_.at<double>(i, 3);
  }

  cv::Mat b_vec = cv::Mat::zeros(3, 1, CV_64F);
  b_vec.at<double>(0) = ud.x;
  b_vec.at<double>(1) = ud.y;
  b_vec.at<double>(2) = 1.0;

  cv::Mat solution;
  if (!cv::solve(eq, b_vec, solution, cv::DECOMP_SVD))
  {
    printf("Camera: imageToWorld 求解失败\n");
    return false;
  }

  double s = solution.at<double>(2);
  if (std::abs(s) < 1e-10)
  {
    printf("Camera: imageToWorld 奇异解（目标可能在相机后方）\n");
    return false;
  }
  x = solution.at<double>(0) / s;
  y = solution.at<double>(1) / s;

  return true;
}

// 世界坐标转像素坐标
bool Camera::transform(cv::Point3d world_point, cv::Point2i &img_point) const
{
  if (!is_calibrated_)
    return false;

  std::vector<cv::Point3d> pts_in = {world_point};
  std::vector<cv::Point2d> pts_out;
  cv::projectPoints(pts_in, rotation_vector_, translation_vector_,
                    intrinsic_matrix_, distortion_coefficients_, pts_out);

  img_point.x = cvRound(pts_out[0].x);
  img_point.y = cvRound(pts_out[0].y);
  return true;
}

// 在图像上绘制世界坐标系三轴
cv::Mat Camera::drawCoordinateSystem(const cv::Mat &img, double axis_length,
                                     int line_thickness) const
{
  if (!is_calibrated_)
    return img.clone();

  cv::Mat result = img.clone();

  cv::Point2i origin_px, x_px, y_px, z_px;
  transform({0, 0, 0}, origin_px);
  transform({axis_length, 0, 0}, x_px);
  transform({0, axis_length, 0}, y_px);
  transform({0, 0, axis_length}, z_px);

  cv::arrowedLine(result, origin_px, x_px, cv::Scalar(0, 0, 255), line_thickness);
  cv::arrowedLine(result, origin_px, y_px, cv::Scalar(255, 0, 0), line_thickness);
  cv::arrowedLine(result, origin_px, z_px, cv::Scalar(255, 0, 255), line_thickness);

  cv::putText(result, "X", x_px, cv::FONT_HERSHEY_SIMPLEX, 0.7,
              cv::Scalar(0, 0, 255), line_thickness);
  cv::putText(result, "Y", y_px, cv::FONT_HERSHEY_SIMPLEX, 0.7,
              cv::Scalar(255, 0, 0), line_thickness);
  cv::putText(result, "Z", z_px, cv::FONT_HERSHEY_SIMPLEX, 0.7,
              cv::Scalar(255, 0, 255), line_thickness);

  return result;
}
