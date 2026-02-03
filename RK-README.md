# RK-YOLO

## 一、环境搭建

### 1.1 基础环境

#### 1.1.1 APT

> 一行完成安装

```shell
sudo apt-get install libxslt1-dev zlib1g-dev libglib2.0 libsm6 \
libgl1-mesa-glx libprotobuf-dev gcc
```



#### <del>1.1.2 conda</del>

下载：

```http
wget https://mirrors.tuna.tsinghua.edu.cn/anaconda/miniconda/Miniconda3-py310_23.11.0-2-Linux-aarch64.sh
```

安装：

```bash
bash Miniconda3-py310_23.11.0-2-Linux-aarch64.sh -b -p ~/Development/miniconda
cd ~/Development/miniconda/bin
./conda init
source ~/.bashrc
```

更新源：

```bash
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main/
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/free/
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/r/
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge/
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/msys2/
```

环境搭建：

```bash
conda create -n yolov5 python=3.8 && conda activate yolov5
```



#### 1.1.3 pip

更新源：

```bash
pip3 config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
```



---



### 1.2 CMAKE

> 最好用Ubuntu来进行onnx到rknn的转化，因为RK需要安装cmake两次，且安装很慢；
>
> 在RK板子上进行转译，cmake版本过高，安装转译依赖时会不兼容；
>
> 无虚拟/物理机条件的，可参考1.2.2，先安装低版本cmake，转换完rknn再卸载安装高版本即可
>
> RK板端进行转换，转换完编译RK-YOLO时，需要高版本(3.10+)才可以编译

#### 1.2.1 软件包安装

> 目前有BUG，版本过高有些内置功能废弃，无法编译

```shell
sudo apt update
sudo apt install cmake
```



#### 1.2.2 源码安装

> RK板端转换rknn，测试版本cmakev3.5.2可行

下载：

```http
https://cmake.org/files/
```

安装编译工具：

```bash
sudo apt update && sudo apt install build-essential
```

按序执行：

> 解压、配置bianyi参数、编译、安装、验证

```bash
tar -zxvf cmake_xxx
cd cmake_xxx
./bootstrap
make -j$(nproc)
sudo make install
cmake --version
```



#### 1.2.3 卸载

进入cmake文件夹下，执行：

```bash
sudo make uninstall
```

检查：

```bash
which cmake
cmake --version
```



---



### 1.2 OpenCV

#### 1.2.1 源码下载

> 下载发行版，版本4.10即可

官网：

```http
https://opencv.org/releases/
```



#### 1.2.2 cmake编译

```shell
unzip opencv-4.10.03

cd opencv-xxxx
mkdir build

sudo apt-get install build-essential libgtk2.0-dev libavcodec-dev libavformat-dev libjpeg.dev libtiff5.dev libswscale-dev

cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
```



#### 1.2.3 OpenCV编译

这一步可能要半小时到一个小时以上甚至更久，有时可能假死，并非无响应，编译过程禁止强制关闭终端。

```shell
sudo make
```



#### 1.2.4 配置环境

> 可选

配置动态链接器：

```shell
sudo gedit /etc/ld.so.conf.d/opencv.conf
```

更新动态链接库：

```shell
sudo ldconfig
```



#### 1.2.5 配置终端环境变量

```shell
sudo gedit /etc/bash.bashrc
```

将下面两行放到最后，fi的下面添加：

```bash
PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig
export PKG_CONFIG_PATH
```

更新更改：

```shell
sudo updatedb
```



### 1.3 YOLO

#### 1.3.1 拉取克隆

拉rknn分支，仓库地址：

```http
https://github.com/BLACKHKER/yolov5.git
```



#### 1.3.2 预训练权重下载

下载好后放在根目录

```http
https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt
```



#### 1.3.3 依赖下载

> 默认已安装好CUDA，参考版本v12.4

<font color="#ff4400">按序</font>执行requirements.txt中的五行命令，安装好后运行detect.py检查环境

![image-20260130170829100](https://typora-picture-zhao.oss-cn-beijing.aliyuncs.com/Typora/image-20260130170829100.png)





## 二、 权重适配

### 2.1 pt转onnx

> Windows PC

YOLO运行export.py，生成yolov5s.onnx

![image-20260130171246990](https://typora-picture-zhao.oss-cn-beijing.aliyuncs.com/Typora/image-20260130171246990.png)



---



### 2.2 onnx转rknn

> Ubuntu x86 PC

#### 2.2.1 依赖下载

执行：

```bash
cd ~/Development/PyCharmWorkspace/rknn-toolkit2/rknn-toolkit2/packages/arm64
pip install numpy pybind11-3.0.1
pip install -r arm64_requirements_cp38.txt
pip install rknn_toolkit2-2.3.2-cp38-cp38-manylinux_2_17_aarch64.manylinux2014_aarch64.whl
```



#### 2.2.2 类型转换

修改test.py : 242，将3566改为要运行的平台，例如3588

![image-20260130202629028](https://typora-picture-zhao.oss-cn-beijing.aliyuncs.com/Typora/image-20260130202629028.png)



---



### 2.3 转型rknn

> Ubuntu arrch64 Lubancat

将onnx放入该文件，运行：

```bash
cd ~/Development/PyCharmWorkspace/rknn-toolkit2/rknn-toolkit2/examples/onnx/yolov5
python3 test.py
```





## 三、部署RKNN

### 3.1 拉取克隆

```http
git clone https://gitee.com/blackhker/lubancat_yolo.git
```



---



### 3.2 编译

> 拉取以后进入/example/yolov5/cpp

```bash
sudo apt install libopencv-dev
sudo apt-get install libsndfile1-dev
./build-linux.sh -t rk3588
```

编译完成会生成/install/rk3588_linux文件夹，将rknn权重放在该文件夹下，运行：

```bash
./yolov5_videocapture_demo xxxx.rknn 1
```





## 四、性能测试

> 所有数据基于yolov5在USB摄像头直连情况下测试，简单场景(1-2个目标)到复杂场景(3个目标左右)；
>
> 最差情况检测到多个高置信度目标时，推理耗时会更长，帧率更低，但不低于20FPS
>
> RTSP视频流推流可能会导致帧率更低；

### 4.1 推理性能

#### 4.1.1 帧率

| 分辨率     | 最低FPS | 最高FPS | 稳定区间 |
| ---------- | ------- | ------- | -------- |
| 640 × 480  | 28      | 42      | 30       |
| 1280 × 960 | 22.5    | 38.8    | 29.2     |



#### 4.1.2 单帧推理耗时

| 分辨率     | 预处理缩放     | 最高耗时(ms) | 最低耗时(ms) | 平均耗时(ms) |
| ---------- | -------------- | ------------ | ------------ | ------------ |
| 640 × 480  | 1(640 × 480)   | 43.443       | 25.781       | 31.2         |
| 1280 × 960 | 0.5(640 × 480) | 44.53        | 25.78        | 34.2         |

