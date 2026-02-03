#!/bin/bash

set -e

eval "~/Development/VisualStudioWorkspace/lubancat_ai_manual_code_cp/example/yolov5/cpp/build-linux.sh -t rk3588"
eval "cp ~/Desktop/yolov5s_relu.rknn ~/Development/VisualStudioWorkspace/lubancat_ai_manual_code_cp/example/yolov5/cpp/install/rk3588_linux/"

