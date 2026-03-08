#!/bin/bash

set -e

eval "~/Development/VisualStudioWorkspace/alubancat_yolo/yolov5/cpp/build-linux.sh -t rk3588"
eval "cp ~/Desktop/yolov5s_relu.rknn ~/Development/VisualStudioWorkspace/alubancat_yolo/yolov5/cpp/install/rk3588_linux/"
eval "cp ~/Desktop/world_params.csv ~/Development/VisualStudioWorkspace/alubancat_yolo/yolov5/cpp/install/rk3588_linux/"

