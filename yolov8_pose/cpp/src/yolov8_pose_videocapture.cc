#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <cmath>
#include <deque>

#include "yolov8_pose.h"
#include "file_utils.h"
#include "image_utils.h"
#include "camera.h"

#include <opencv2/opencv.hpp>
#include <mosquitto.h>

#define KP_LEFT_SHOULDER 5
#define KP_RIGHT_SHOULDER 6
#define KP_LEFT_WRIST 9
#define KP_RIGHT_WRIST 10
#define KP_LEFT_HIP 11
#define KP_RIGHT_HIP 12

// 坐标输出单位: 1.0=mm, 10.0=cm, 1000.0=m
#define COORD_UNIT_DIVISOR  10.0
#define COORD_UNIT_NAME     "cm"

// BOX_H_MIN: 最远处AGV的bbox像素高度
// BOX_H_MAX: 最近处AGV的bbox像素高度
#define AGV_BOX_H_MIN  15.0f
#define AGV_BOX_H_MAX  70.0f

#define RAISE_HAND_Y_THRESH 30.0f
#define MOVE_DISTANCE_THRESH 15.0f
#define HISTORY_FRAME_COUNT 5
#define KP_CONF_THRESH 0.5f

// 动作枚举
enum Action
{
  ACTION_STILL = 0,  // 静止
  ACTION_MOVING,     // 运动
  ACTION_RAISE_HAND, // 举手
};

static const char *action_names[] = {"Still", "Moving", "Raise Hand"};
static const cv::Scalar action_colors[] = {
    cv::Scalar(0, 255, 0),   // 静止 - 绿色
    cv::Scalar(0, 165, 255), // 运动 - 橙色
    cv::Scalar(0, 0, 255),   // 举手 - 红色
};

// 每个检测目标的历史髋部中心位置
static std::deque<cv::Point2f> hip_history;

// 判断单个关键点是否有效
static inline bool kp_valid(const float kp[3])
{
  return kp[2] >= KP_CONF_THRESH && kp[0] != 0 && kp[1] != 0;
}

// 计算髋部中心点
static bool get_hip_center(const object_detect_result *det, cv::Point2f &center)
{
  bool l_valid = kp_valid(det->keypoints[KP_LEFT_HIP]);
  bool r_valid = kp_valid(det->keypoints[KP_RIGHT_HIP]);

  if (l_valid && r_valid)
  {
    center.x = (det->keypoints[KP_LEFT_HIP][0] + det->keypoints[KP_RIGHT_HIP][0]) / 2.0f;
    center.y = (det->keypoints[KP_LEFT_HIP][1] + det->keypoints[KP_RIGHT_HIP][1]) / 2.0f;
    return true;
  }
  else if (l_valid)
  {
    center.x = det->keypoints[KP_LEFT_HIP][0];
    center.y = det->keypoints[KP_LEFT_HIP][1];
    return true;
  }
  else if (r_valid)
  {
    center.x = det->keypoints[KP_RIGHT_HIP][0];
    center.y = det->keypoints[KP_RIGHT_HIP][1];
    return true;
  }
  return false;
}

// 判断是否举手：手腕y坐标明显小于/高于肩膀y坐标
static bool is_raising_hand(const object_detect_result *det)
{
  // 左手举起
  if (kp_valid(det->keypoints[KP_LEFT_WRIST]) && kp_valid(det->keypoints[KP_LEFT_SHOULDER]))
  {
    if (det->keypoints[KP_LEFT_SHOULDER][1] - det->keypoints[KP_LEFT_WRIST][1] > RAISE_HAND_Y_THRESH)
    {
      return true;
    }
  }
  // 右手举起
  if (kp_valid(det->keypoints[KP_RIGHT_WRIST]) && kp_valid(det->keypoints[KP_RIGHT_SHOULDER]))
  {
    if (det->keypoints[KP_RIGHT_SHOULDER][1] - det->keypoints[KP_RIGHT_WRIST][1] > RAISE_HAND_Y_THRESH)
    {
      return true;
    }
  }
  return false;
}

// 判断是否在运动：通过历史帧髋部中心的位移判断
static bool is_moving(const cv::Point2f &current_hip)
{
  if (hip_history.size() < 2)
  {
    return false;
  }
  // 计算当前位置与历史最早位置的距离
  const cv::Point2f &oldest = hip_history.front();
  float dx = current_hip.x - oldest.x;
  float dy = current_hip.y - oldest.y;
  float dist = std::sqrt(dx * dx + dy * dy);
  return dist > MOVE_DISTANCE_THRESH;
}

// 综合判断动作 优先级：举手>运动>静止
static Action classify_action(const object_detect_result *det)
{
  // 举手判断
  if (is_raising_hand(det))
  {
    return ACTION_RAISE_HAND;
  }

  // 运动判断
  cv::Point2f hip_center;
  if (get_hip_center(det, hip_center))
  {
    bool moving = is_moving(hip_center);
    // 更新历史
    hip_history.push_back(hip_center);
    if ((int)hip_history.size() > HISTORY_FRAME_COUNT)
    {
      hip_history.pop_front();
    }
    if (moving)
    {
      return ACTION_MOVING;
    }
  }

  // 默认静止
  return ACTION_STILL;
}

static const unsigned char box_colors[19][3] = {
    {54, 67, 244},
    {99, 30, 233},
    {176, 39, 156},
    {183, 58, 103},
    {181, 81, 63},
    {243, 150, 33},
    {244, 169, 3},
    {212, 188, 0},
    {136, 150, 0},
    {80, 175, 76},
    {74, 195, 139},
    {57, 220, 205},
    {59, 235, 255},
    {7, 193, 255},
    {0, 152, 255},
    {34, 87, 255},
    {72, 85, 121},
    {158, 158, 158},
    {139, 125, 96}};

static void draw_pose(cv::Mat &image, const object_detect_result *det_result)
{

  std::vector<cv::Scalar> pose_palette = {
      {255, 128, 0}, {255, 153, 51}, {255, 178, 102}, {230, 230, 0}, {255, 153, 255}, {153, 204, 255}, {255, 102, 255}, {255, 51, 255}, {102, 178, 255}, {51, 153, 255}, {255, 153, 153}, {255, 102, 102}, {255, 51, 51}, {153, 255, 153}, {102, 255, 102}, {51, 255, 51}, {0, 255, 0}, {0, 0, 255}, {255, 0, 0}, {255, 255, 255}};

  std::vector<cv::Point> skeleton = {
      {15, 13}, {13, 11}, {16, 14}, {14, 12}, {11, 12}, {5, 11}, {6, 12}, {5, 6}, {5, 7}, {6, 8}, {7, 9}, {8, 10}, {1, 2}, {0, 1}, {0, 2}, {1, 3}, {2, 4}, {3, 5}, {4, 6}};

  std::vector<cv::Scalar> limb_color = {
      pose_palette[9], pose_palette[9], pose_palette[9], pose_palette[9], pose_palette[7],
      pose_palette[7], pose_palette[7], pose_palette[0], pose_palette[0], pose_palette[0],
      pose_palette[0], pose_palette[0], pose_palette[16], pose_palette[16], pose_palette[16],
      pose_palette[16], pose_palette[16], pose_palette[16], pose_palette[16]};

  std::vector<cv::Scalar> kpt_color = {
      pose_palette[16], pose_palette[16], pose_palette[16], pose_palette[16], pose_palette[16],
      pose_palette[0], pose_palette[0], pose_palette[0], pose_palette[0], pose_palette[0],
      pose_palette[0], pose_palette[9], pose_palette[9], pose_palette[9], pose_palette[9],
      pose_palette[9], pose_palette[9]};

  for (int i = 0; i < 17; ++i)
  {
    if (det_result->keypoints[i][2] < 0.5)
      continue;
    if (det_result->keypoints[i][0] != 0 && det_result->keypoints[i][1] != 0)
      cv::circle(image, cv::Point(det_result->keypoints[i][0], det_result->keypoints[i][1]), 2, kpt_color[i], -1, cv::LINE_AA);
  }

  for (int i = 0; i < (int)skeleton.size(); ++i)
  {
    auto &index = skeleton[i];
    int x = index.x;
    int y = index.y;

    if (det_result->keypoints[x][2] < 0.5 || det_result->keypoints[y][2] < 0.5)
      continue;
    if (det_result->keypoints[x][0] == 0 || det_result->keypoints[x][1] == 0 || det_result->keypoints[y][0] == 0 || det_result->keypoints[y][1] == 0)
      continue;
    cv::line(image, cv::Point(det_result->keypoints[x][0], det_result->keypoints[x][1]),
             cv::Point(det_result->keypoints[y][0], det_result->keypoints[y][1]), limb_color[i], 1, cv::LINE_AA);
  }
}

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    printf("Usage: %s <model path> <camera device id/video path> [world_params.csv] [--board-id <id>] [--debug]\n", argv[0]);
    printf("Usage: %s  yolov8_pose.rknn  0\n", argv[0]);
    printf("Usage: %s  yolov8_pose.rknn  0  world_params.csv\n", argv[0]);
    printf("Usage: %s  yolov8_pose.rknn  0  world_params.csv  --board-id rk_01  --debug\n", argv[0]);
    printf("--board-id: 板子唯一标识，多相机部署时区分数据来源\n");
    printf("--debug:    在画面上显示AGV坐标取点位置\n");
    return -1;
  }

  const char *model_path = argv[1];
  const char *device_name = argv[2];

  // 解析可选参数
  bool debug_point = false;
  const char *board_id = "rk_00";  // 默认board_id
  for (int i = 3; i < argc; i++) {
    if (strcmp(argv[i], "--debug") == 0) {
      debug_point = true;
      printf("调试模式：显示AGV取点位置\n");
    } else if (strcmp(argv[i], "--board-id") == 0 && i + 1 < argc) {
      board_id = argv[i + 1];
      i++;  // 跳过下一个参数(board_id的值)
      printf("board_id: %s\n", board_id);
    }
  }

  // 相机内外参
  Camera camera;
  if (argc > 3 && argv[3][0] != '-') {
    if (!camera.loadFromCSV(argv[3])) {
      printf("加载相机参数失败，不进行坐标转换\n");
    } else {
      printf("相机参数加载成功，转相对世界坐标\n");
    }
  }

  const char *mqtt_broker = "8.137.120.144";
  int mqtt_port = 1883;
  struct mosquitto *mosq = nullptr;
  mosquitto_lib_init();
  mosq = mosquitto_new("yolov8_pose_rk3588", true, nullptr);
  if (!mosq) {
    printf("MQTT: 创建客户端失败\n");
  } else if (mosquitto_connect(mosq, mqtt_broker, mqtt_port, 60) != MOSQ_ERR_SUCCESS) {
    printf("MQTT: 连接%s:%d失败，将不发送坐标\n", mqtt_broker, mqtt_port);
    mosquitto_destroy(mosq);
    mosq = nullptr;
  } else {
    mosquitto_loop_start(mosq);
    printf("MQTT: 已连接%s:%d\n", mqtt_broker, mqtt_port);
  }

  int ret;
  cv::Mat frame, image;
  struct timeval start_time, stop_time;
  rknn_app_context_t rknn_app_ctx;
  image_buffer_t src_image;

  memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));
  memset(&src_image, 0, sizeof(image_buffer_t));

  cv::VideoCapture cap;
  if (isdigit(device_name[0]))
  {
    // 摄像头
    int camera_id = atoi(argv[2]);
    cap.open(camera_id);

    if (!cap.isOpened())
    {
      printf("Error: Could not open camera\n");
      return -1;
    }
    // cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  }
  else
  {
    // 视频文件或者其他
    cap.open(argv[2]);
    if (!cap.isOpened())
    {
      printf("Error: Could not open video file\n");
      return -1;
    }
  }
  int frame_count = 0;
  struct timeval frame_start, frame_end;

  init_post_process();
  ret = init_yolov8_pose_model(model_path, &rknn_app_ctx);
  if (ret != 0)
  {
    printf("init_yolov8_pose_model fail! ret=%d model_path=%s\n", ret, model_path);
    goto out;
  }

  while (true)
  {
    gettimeofday(&frame_start, NULL);

    if (!cap.read(frame))
    {
      printf("cap read frame fail!\n");
      break;
    }

    cv::cvtColor(frame, image, cv::COLOR_BGR2RGB);
    src_image.width = image.cols;
    src_image.height = image.rows;
    src_image.format = IMAGE_FORMAT_RGB888;
    src_image.virt_addr = (unsigned char *)image.data;

    // rknn inference and postprocess
    object_detect_result_list od_results;
    ret = inference_yolov8_pose_model(&rknn_app_ctx, &src_image, &od_results);
    if (ret != 0)
    {
      printf("init_yolov10_model fail! ret=%d\n", ret);
      goto out;
    }

    // 每帧开始前清空历史(仅对第一个检测到的人做动作分类)
    // 如果需要多人分别追踪，需要为每个人维护独立的历史
    int color_index = 0;
    char text[256];
    for (int i = 0; i < od_results.count; i++)
    {
      const unsigned char *color = box_colors[color_index % 19];
      cv::Scalar cc(color[0], color[1], color[2]);
      color_index++;

      object_detect_result *det_result = &(od_results.results[i]);
      printf("%s @ (%d %d %d %d) %.3f\n", coco_cls_to_name(det_result->cls_id),
             det_result->box.left, det_result->box.top,
             det_result->box.right, det_result->box.bottom,
             det_result->prop);
      sprintf(text, "%s %.1f%%", coco_cls_to_name(det_result->cls_id), det_result->prop * 100);

      cv::rectangle(frame, cv::Rect(cv::Point(det_result->box.left, det_result->box.top), cv::Point(det_result->box.right, det_result->box.bottom)), cc, 2);

      int baseLine = 0;
      cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

      int x = det_result->box.left;
      int y = det_result->box.top - label_size.height - baseLine;
      if (y < 0)
        y = 0;
      if (x + label_size.width > frame.cols)
        x = frame.cols - label_size.width;

      cv::rectangle(frame, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)), cc, -1);
      cv::putText(frame, text, cv::Point(x, y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));

      draw_pose(frame, det_result);

      // 动作分类和显示 仅person，cls_id=0
      const char *action_name = "none";
      if (det_result->cls_id == 0) {
        Action action = classify_action(det_result);
        action_name = action_names[action];
        cv::Scalar action_color = action_colors[action];

        int action_y = det_result->box.bottom + 20;
        if (action_y > frame.rows - 10)
          action_y = det_result->box.bottom - 10;

        cv::putText(frame, action_name, cv::Point(det_result->box.left, action_y),
                    cv::FONT_HERSHEY_SIMPLEX, 1, action_color, 2, cv::LINE_AA);
        printf("Action: %s\n", action_name);
      }

      // 世界坐标转换
      if (camera.isCalibrated()) {
        float pixel_x, pixel_y;
        if (det_result->cls_id == 0 &&
            kp_valid(det_result->keypoints[15]) && kp_valid(det_result->keypoints[16])) {
          pixel_x = (det_result->keypoints[15][0] + det_result->keypoints[16][0]) / 2.0f;
          pixel_y = (det_result->keypoints[15][1] + det_result->keypoints[16][1]) / 2.0f;
        } else {
          // 根据bbox高度自适应取点(近处靠近中心远处靠近底边)
          float box_h = det_result->box.bottom - det_result->box.top;
          float box_center_y = (det_result->box.top + det_result->box.bottom) / 2.0f;
          float box_bottom_y = det_result->box.bottom;

          float alpha = (box_h - AGV_BOX_H_MIN) / (AGV_BOX_H_MAX - AGV_BOX_H_MIN);
          alpha = std::max(0.0f, std::min(1.0f, alpha));

          pixel_x = (det_result->box.left + det_result->box.right) / 2.0f;
          pixel_y = alpha * box_center_y + (1.0f - alpha) * box_bottom_y;

          if (debug_point) {
            int cx = (int)pixel_x;
            int py = (int)pixel_y;
            int cy = (int)box_center_y;
            int by = (int)box_bottom_y;

            // 画实心圆标记取点位置（黄色）
            cv::circle(frame, cv::Point(cx, py), 6, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
            // 画空心圆标记中心点（绿色）
            cv::circle(frame, cv::Point(cx, cy), 4, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            // 画空心圆标记底边点（红色）
            cv::circle(frame, cv::Point(cx, by), 4, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
            // 连线显示插值效果
            cv::line(frame, cv::Point(cx, cy), cv::Point(cx, by),
                     cv::Scalar(128, 128, 128), 1, cv::LINE_AA);
            // 标注alpha和box_h
            char alpha_text[32];
            snprintf(alpha_text, sizeof(alpha_text), "a=%.2f h=%.0f", alpha, box_h);
            cv::putText(frame, alpha_text,
                        cv::Point(det_result->box.right + 3, py),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);
            printf("[AGV] box_h=%.0f alpha=%.2f pixel_y=%.0f (center=%.0f bottom=%.0f)\n",
                   box_h, alpha, pixel_y, box_center_y, box_bottom_y);
          }
        }

        double world_x, world_y;
        if (camera.imageToWorld({pixel_x, pixel_y}, 0.0, world_x, world_y)) {
          // 单位转换
          double out_x = world_x / COORD_UNIT_DIVISOR;
          double out_y = world_y / COORD_UNIT_DIVISOR;

          char coord_text[64];
          snprintf(coord_text, sizeof(coord_text), "W:(%.1f, %.1f)%s", out_x, out_y, COORD_UNIT_NAME);
          cv::putText(frame, coord_text,
                      cv::Point(det_result->box.left, det_result->box.bottom + 40),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
          printf("%s World: (%.1f, %.1f) %s\n", coco_cls_to_name(det_result->cls_id), out_x, out_y, COORD_UNIT_NAME);

          // MQTT发布: person和agv分不同主题，附带board_id和毫秒时间戳
          if (mosq) {
            char payload[512];
            struct timeval tv_now;
            gettimeofday(&tv_now, NULL);
            long long ts_ms = (long long)tv_now.tv_sec * 1000 + tv_now.tv_usec / 1000;

            if (det_result->cls_id == 0) {
              snprintf(payload, sizeof(payload),
                       "{\"board_id\":\"%s\",\"timestamp\":%lld,\"x\":%.2f,\"y\":%.2f,\"unit\":\"%s\",\"action\":\"%s\",\"conf\":%.2f}",
                       board_id, ts_ms, out_x, out_y, COORD_UNIT_NAME, action_name, det_result->prop);
              mosquitto_publish(mosq, nullptr, "yolov8_pose/person",
                                strlen(payload), payload, 0, false);
            } else {
              snprintf(payload, sizeof(payload),
                       "{\"board_id\":\"%s\",\"timestamp\":%lld,\"x\":%.2f,\"y\":%.2f,\"unit\":\"%s\",\"conf\":%.2f}",
                       board_id, ts_ms, out_x, out_y, COORD_UNIT_NAME, det_result->prop);
              mosquitto_publish(mosq, nullptr, "yolov8_pose/agv",
                                strlen(payload), payload, 0, false);
            }
          }
        }
      }
    }

    // 如果没有检测到人，清空历史
    if (od_results.count == 0)
    {
      hip_history.clear();
    }

    // 绘制世界坐标系三轴(X红/Y蓝/Z紫)
    if (camera.isCalibrated()) {
      frame = camera.drawCoordinateSystem(frame, 500.0, 2);
    }

    // FPS统计
    gettimeofday(&frame_end, NULL);
    double frame_ms = (frame_end.tv_sec - frame_start.tv_sec) * 1000.0 +
                      (frame_end.tv_usec - frame_start.tv_usec) / 1000.0;
    double fps = 1000.0 / frame_ms;
    frame_count++;

    char fps_text[64];
    snprintf(fps_text, sizeof(fps_text), "FPS: %.1f  (%.1fms)", fps, frame_ms);
    cv::putText(frame, fps_text, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);

    // 每30帧打印一次性能汇总
    if (frame_count % 30 == 0) {
      printf("=====性能:FPS=%.1f, 端到端=%.1fms, 检测数=%d====\n",
             fps, frame_ms, od_results.count);
    }

    // 显示结果
    cv::imshow("yolov8_pose_videocapture", frame);

    char c = cv::waitKey(1);
    if (c == 27)
    { // ESC
      break;
    }
  }

out:
  deinit_post_process();
  ret = release_yolov8_pose_model(&rknn_app_ctx);
  if (ret != 0)
  {
    printf("release_yolov8_pose_model fail! ret=%d\n", ret);
  }

  // MQTT 清理
  if (mosq) {
    mosquitto_loop_stop(mosq, true);
    mosquitto_destroy(mosq);
  }
  mosquitto_lib_cleanup();

  return 0;
}
