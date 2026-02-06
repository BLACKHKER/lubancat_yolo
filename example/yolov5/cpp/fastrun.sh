#!/bin/bash

set -e

eval "~/Development/VisualStudioWorkspace/alubancat_yolo/example/yolov5/cpp/build-linux.sh -t rk3588"
eval "cp ~/Desktop/yolov5s_relu.rknn ~/Development/VisualStudioWorkspace/alubancat_yolo/example/yolov5/cpp/install/rk3588_linux/"

