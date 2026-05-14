// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*-------------------------------------------
                Includes
-------------------------------------------*/
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

#include <opencv2/opencv.hpp>

#define KP_LEFT_SHOULDER   5
#define KP_RIGHT_SHOULDER  6
#define KP_LEFT_WRIST      9
#define KP_RIGHT_WRIST     10
#define KP_LEFT_HIP        11
#define KP_RIGHT_HIP       12

// 举手判断：手腕需要高于肩膀多少像素才算举手
#define RAISE_HAND_Y_THRESH  30.0f
// 运动判断：髋部中心在连续帧之间的位移（像素）超过此值认为在运动
#define MOVE_DISTANCE_THRESH 15.0f
// 保存历史帧数，用于运动检测的滑动窗口
#define HISTORY_FRAME_COUNT  5
// 关键点置信度阈值
#define KP_CONF_THRESH       0.5f

// 动作枚举
enum Action {
    ACTION_STILL = 0,   // 静止
    ACTION_MOVING,      // 运动
    ACTION_RAISE_HAND,  // 举手
};

static const char* action_names[] = {"Still", "Moving", "Raise Hand"};
static const cv::Scalar action_colors[] = {
    cv::Scalar(0, 255, 0),    // 静止 - 绿色
    cv::Scalar(0, 165, 255),  // 运动 - 橙色
    cv::Scalar(0, 0, 255),    // 举手 - 红色
};

// 每个检测目标的历史髋部中心位置（用于运动检测）
static std::deque<cv::Point2f> hip_history;

// 判断单个关键点是否有效
static inline bool kp_valid(const float kp[3]) {
    return kp[2] >= KP_CONF_THRESH && kp[0] != 0 && kp[1] != 0;
}

// 计算髋部中心点
static bool get_hip_center(const object_detect_result *det, cv::Point2f &center) {
    bool l_valid = kp_valid(det->keypoints[KP_LEFT_HIP]);
    bool r_valid = kp_valid(det->keypoints[KP_RIGHT_HIP]);

    if (l_valid && r_valid) {
        center.x = (det->keypoints[KP_LEFT_HIP][0] + det->keypoints[KP_RIGHT_HIP][0]) / 2.0f;
        center.y = (det->keypoints[KP_LEFT_HIP][1] + det->keypoints[KP_RIGHT_HIP][1]) / 2.0f;
        return true;
    } else if (l_valid) {
        center.x = det->keypoints[KP_LEFT_HIP][0];
        center.y = det->keypoints[KP_LEFT_HIP][1];
        return true;
    } else if (r_valid) {
        center.x = det->keypoints[KP_RIGHT_HIP][0];
        center.y = det->keypoints[KP_RIGHT_HIP][1];
        return true;
    }
    return false;
}

// 判断是否举手：手腕 y 坐标明显小于（高于）肩膀 y 坐标
static bool is_raising_hand(const object_detect_result *det) {
    // 左手举起
    if (kp_valid(det->keypoints[KP_LEFT_WRIST]) && kp_valid(det->keypoints[KP_LEFT_SHOULDER])) {
        if (det->keypoints[KP_LEFT_SHOULDER][1] - det->keypoints[KP_LEFT_WRIST][1] > RAISE_HAND_Y_THRESH) {
            return true;
        }
    }
    // 右手举起
    if (kp_valid(det->keypoints[KP_RIGHT_WRIST]) && kp_valid(det->keypoints[KP_RIGHT_SHOULDER])) {
        if (det->keypoints[KP_RIGHT_SHOULDER][1] - det->keypoints[KP_RIGHT_WRIST][1] > RAISE_HAND_Y_THRESH) {
            return true;
        }
    }
    return false;
}

// 判断是否在运动：通过历史帧髋部中心的位移判断
static bool is_moving(const cv::Point2f &current_hip) {
    if (hip_history.size() < 2) {
        return false;
    }
    // 计算当前位置与历史最早位置的距离
    const cv::Point2f &oldest = hip_history.front();
    float dx = current_hip.x - oldest.x;
    float dy = current_hip.y - oldest.y;
    float dist = std::sqrt(dx * dx + dy * dy);
    return dist > MOVE_DISTANCE_THRESH;
}

// 综合判断动作（优先级：举手 > 运动 > 静止）
static Action classify_action(const object_detect_result *det) {
    // 1. 先判断举手
    if (is_raising_hand(det)) {
        return ACTION_RAISE_HAND;
    }

    // 2. 再判断运动
    cv::Point2f hip_center;
    if (get_hip_center(det, hip_center)) {
        bool moving = is_moving(hip_center);
        // 更新历史
        hip_history.push_back(hip_center);
        if ((int)hip_history.size() > HISTORY_FRAME_COUNT) {
            hip_history.pop_front();
        }
        if (moving) {
            return ACTION_MOVING;
        }
    }

    // 3. 默认静止
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
    {139, 125, 96}
};

static void draw_pose(cv::Mat& image, const object_detect_result *det_result){

    std::vector<cv::Scalar> pose_palette = {
        {255, 128,   0}, {255, 153,  51}, {255, 178, 102}, {230, 230,   0}, {255, 153, 255},
        {153, 204, 255}, {255, 102, 255}, {255, 51,  255}, {102, 178, 255}, {51,  153, 255},
        {255, 153, 153}, {255, 102, 102}, {255, 51,   51}, {153, 255, 153}, {102, 255, 102},
        {51,  255,  51}, {0,   255,   0}, {0,   0,   255}, {255, 0,     0}, {255, 255, 255}
    };

    std::vector<cv::Point> skeleton = {
        {15, 13}, {13, 11}, {16, 14}, {14, 12}, {11, 12}, {5, 11}, {6, 12},
        {5,   6}, {5,   7}, {6,   8}, {7,   9}, {8,  10}, {1,  2}, {0,  1},
        {0,   2}, {1,   3}, {2,   4}, {3,   5}, {4,   6}
    };

    std::vector<cv::Scalar> limb_color = {
        pose_palette[9],  pose_palette[9],  pose_palette[9],  pose_palette[9],  pose_palette[7],
        pose_palette[7],  pose_palette[7],  pose_palette[0],  pose_palette[0],  pose_palette[0],
        pose_palette[0],  pose_palette[0],  pose_palette[16], pose_palette[16], pose_palette[16],
        pose_palette[16], pose_palette[16], pose_palette[16], pose_palette[16]
    };

    std::vector<cv::Scalar> kpt_color = {
        pose_palette[16], pose_palette[16], pose_palette[16], pose_palette[16], pose_palette[16],
        pose_palette[0],  pose_palette[0],  pose_palette[0],  pose_palette[0],  pose_palette[0],
        pose_palette[0],  pose_palette[9],  pose_palette[9],  pose_palette[9],  pose_palette[9],
        pose_palette[9],  pose_palette[9]
    };

    for(int i = 0; i < 17; ++i){
        if(det_result->keypoints[i][2] < 0.5)
            continue;
        if(det_result->keypoints[i][0] != 0 && det_result->keypoints[i][1] != 0)
            cv::circle(image, cv::Point(det_result->keypoints[i][0], det_result->keypoints[i][1]), 2, kpt_color[i], -1, cv::LINE_AA);
    }

    for(int i = 0; i < (int)skeleton.size(); ++i){
        auto& index = skeleton[i];
        int x = index.x;
        int y = index.y;

        if(det_result->keypoints[x][2] < 0.5 || det_result->keypoints[y][2]< 0.5)
            continue;
        if(det_result->keypoints[x][0] == 0 || det_result->keypoints[x][1] == 0 || det_result->keypoints[y][0] == 0 || det_result->keypoints[y][1] == 0)
            continue;
        cv::line(image, cv::Point(det_result->keypoints[x][0], det_result->keypoints[x][1]),
            cv::Point(det_result->keypoints[y][0], det_result->keypoints[y][1]), limb_color[i], 1, cv::LINE_AA);
    }
}

/*-------------------------------------------
                  Main Function
-------------------------------------------*/
int main(int argc, char **argv)
{
    if (argc != 3)
    {
        printf("%s <model path> <camera device id/video path>\n", argv[0]);
        printf("Usage: %s  yolov8_pose.rknn  0 \n", argv[0]);
        printf("Usage: %s  yolov8_pose.rknn /path/xxxx.mp4\n", argv[0]);
        return -1;
    }

    const char *model_path = argv[1];
    const char *device_name = argv[2];

    int ret;
    cv::Mat frame, image;
    struct timeval start_time, stop_time;
    rknn_app_context_t rknn_app_ctx;
    image_buffer_t src_image;

    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));
    memset(&src_image, 0, sizeof(image_buffer_t));

    cv::VideoCapture cap;
    if (isdigit(device_name[0])) {
        // 摄像头
        int camera_id = atoi(argv[2]);
        cap.open(camera_id);

        if (!cap.isOpened()) {
            printf("Error: Could not open camera.\n");
            return -1;
        }
        // cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    } else {
        // 视频文件或者其他
        cap.open(argv[2]);
        if (!cap.isOpened()) {
            printf("Error: Could not open video file.\n");
            return -1;
        }

    }
    init_post_process();
    ret = init_yolov8_pose_model(model_path, &rknn_app_ctx);
    if (ret != 0)
    {
        printf("init_yolov10_model fail! ret=%d model_path=%s\n", ret, model_path);
        goto out;
    }

	while(true) {
        if (!cap.read(frame)) {
            printf("cap read frame fail!\n");
            break;
        }

        cv::cvtColor(frame, image, cv::COLOR_BGR2RGB);
        src_image.width  = image.cols;
        src_image.height = image.rows;
        src_image.format = IMAGE_FORMAT_RGB888;
        src_image.virt_addr = (unsigned char*)image.data;

        // rknn inference and postprocess
        object_detect_result_list od_results;
        ret = inference_yolov8_pose_model(&rknn_app_ctx, &src_image, &od_results);
        if (ret != 0)
        {
            printf("init_yolov10_model fail! ret=%d\n", ret);
            goto out;
        }

        // 每帧开始前清空历史（仅对第一个检测到的人做动作分类）
        // 如果需要多人分别追踪，需要为每个人维护独立的历史
        int color_index = 0;
        char text[256];
        for (int i = 0; i < od_results.count; i++)
        {
            const unsigned char* color = box_colors[color_index % 19];
            cv::Scalar cc(color[0], color[1], color[2]);
            color_index++;

            object_detect_result *det_result = &(od_results.results[i]);
            printf("%s @ (%d %d %d %d) %.3f\n", coco_cls_to_name(det_result->cls_id),
                det_result->box.left, det_result->box.top,
                det_result->box.right, det_result->box.bottom,
                det_result->prop);
            sprintf(text, "%s %.1f%%", coco_cls_to_name(det_result->cls_id), det_result->prop * 100);

            cv::rectangle(frame, cv::Rect(cv::Point(det_result->box.left, det_result->box.top),
                            cv::Point(det_result->box.right, det_result->box.bottom)), cc, 2);

            int baseLine = 0;
            cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

            int x = det_result->box.left;
            int y = det_result->box.top - label_size.height - baseLine;
            if (y < 0)
                y = 0;
            if (x + label_size.width > frame.cols)
                x = frame.cols - label_size.width;

            cv::rectangle(frame, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),cc,-1);
            cv::putText(frame, text, cv::Point(x, y + label_size.height),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));

            draw_pose(frame, det_result);

            // ---- 动作分类 ----
            Action action = classify_action(det_result);
            const char* action_name = action_names[action];
            cv::Scalar action_color = action_colors[action];

            // 在人体框下方显示动作标签
            int action_y = det_result->box.bottom + 20;
            if (action_y > frame.rows - 10)
                action_y = det_result->box.bottom - 10;

            cv::putText(frame, action_name, cv::Point(det_result->box.left, action_y),
                cv::FONT_HERSHEY_SIMPLEX, 1, action_color, 2, cv::LINE_AA);

            printf("Action: %s\n", action_name);
        }

        // 如果没有检测到人，清空历史
        if (od_results.count == 0) {
            hip_history.clear();
        }

        // 显示结果
        cv::imshow("yolov8_pose_videocapture", frame);

        char c = cv::waitKey(1);
		if (c == 27) { // ESC
			break;
		}
    }

out:
    deinit_post_process();
    ret = release_yolov8_pose_model(&rknn_app_ctx);
    if (ret != 0)
    {
        printf("release_yolov10_model fail! ret=%d\n", ret);
    }
    return 0;
}
