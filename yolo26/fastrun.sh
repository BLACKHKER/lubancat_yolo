#!/bin/bash

set -e

eval "~/Development/VisualStudioWorkspace/alubancat_yolo/yolo26/cpp/build-linux.sh -t rk3588"
eval "cp ~/Development/VisualStudioWorkspace/alubancat_yolo/weights/yolo26_i8.rknn ~/Development/VisualStudioWorkspace/alubancat_yolo/yolo26/cpp/install/rk3588_linux"

