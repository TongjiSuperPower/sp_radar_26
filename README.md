# sp_radar_26使用指南
## 编译
- 系统环境Ubuntu22.04，**不要使用Vmware**
- 鱼香ros一键安装ros2

    ```wget http://fishros.com/install -O fishros && . fishros```

- NVIDIA驱动(>=525)、[CUDA Toolkit(12.0)(runfile安装)](https://developer.nvidia.com/cuda-12-0-0-download-archive?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=runfile_local)、[cudnn（8.9.7）（deb安装）](https://developer.nvidia.com/rdp/cudnn-archive)、[tensorrt（8.6.1）(tar安装)](https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/secure/8.6.1/tars/TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-12.0.tar.gz)

    安装CUDA Toolkit时可以选择安装驱动，即可以直接从用runfile安装CUDA Toolkit安装开始，当然你也可以通过Ubuntu的apt或software&updates下载驱动

- 编译时还需要安装以下库

    - [serial](https://blog.csdn.net/weixin_42670590/article/details/137887249)
    ```
    mkdir serial
    git clone https://github.com/ZhaoXiangBox/serial
    或者（git clone https://gitee.com/laiguanren/serial.git）
    cd serial
    mkdir build
    cd build
    cmake ..
    make
    make install 
    ```

    - [small_gicp](https://github.com/koide3/small_gicp)

    ```
    sudo apt-get install libeigen3-dev libomp-dev

    cd small_gicp
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
    sudo make install
    ```

    - [Open3D](https://blog.csdn.net/m0_51661400/article/details/134735883)[（Open3D安装时ZSTD库依赖报错）](https://blog.csdn.net/m0_51661400/article/details/134735883)

    ```
    git clone --recursive https://github.com/intel-isl/Open3D
    cd Open3D
    bash util/install_deps_ubuntu.sh

    cd Open3D
    mkdir build
    cd build
    cmake ..
    # 也可以根据自己的需要改为下面的命令
    # cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=${HOME}/open3d_install ..
    make -j$(nproc)
    sudo make install
    ```

    - [nlohmann_json](https://github.com/nlohmann/json?tab=readme-ov-file#quick-reference)

    ```
    git clone https://github.com/nlohmann/json.git
    cd json
    mkdir build
    cd build/
    cmake ..
    make
    sudo make install
    ```

- 第一次编译先编译radar_msgs包、livox_sdk_vendor和livox_interfaces三个包，编译完记得
```source install/setup.bash```

- 编译detection功能包时，需要增加```--symlink```参数。若运行时遇到动态链接库(libdeploy.so, libMvControl,so等)缺失的问题，都是添加这个参数。

## YOLO模型格式准备
- 识别模块在运行时需要使用YOLO的engine文件，仓库提供了相应的pt文件，需要用如下方式转换
    1. 安装conda
    ```
    wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
    bash Miniconda3-latest-Linux-x86_64.sh
    ```

    2. 创建和配置虚拟环境，具体参考[trtyolo-export](https://github.com/laugh12321/TensorRT-YOLO/tree/export)
    ```
    conda create --name trtyolo python=3.8
    conda activate trtyolo
    pip install trtyolo-export
    ```

    3. 将pt文件转为onnx文件
    ```
    cd src/main/detection/models
    trtyolo export -w car_12s.pt -v ultralytics -o ignore --max_boxes 100 --iou_thres 0.45 --conf_thres 0.25    
    
    # 以及装甲板的模型
    trtyolo export -w armor_11s.pt -v ultralytics -o ignore --max_boxes 100 --iou_thres 0.45 --conf_thres 0.25   
    ```

    - 注：若遇到类似下述报错，参考[博客](https://blog.csdn.net/qq_42730750/article/details/139582293)
    ```
    (trtyolo) radar@radar:~/Desktop/radar26/sp_radar_26/src/main/detection/models$ trtyolo export -w car_11s.pt -v ultralytics -o output --max_boxes 100 --iou_thres 0.45 --conf_thres 0.25 
    Traceback (most recent call last):
    File "/home/radar/miniconda3/envs/trtyolo/bin/trtyolo", line 5, in <module>
        from trtyolo_export.cli import trtyolo
    File "/home/radar/miniconda3/envs/trtyolo/lib/python3.8/site-packages/trtyolo_export/cli.py", line 36, in <module>
        import torch
    File "/home/radar/miniconda3/envs/trtyolo/lib/python3.8/site-packages/torch/__init__.py", line 290, in <module>
        from torch._C import *  # noqa: F403
    ImportError: /home/radar/miniconda3/envs/trtyolo/lib/python3.8/site-packages/torch/lib/../../nvidia/cusparse/lib/libcusparse.so.12: undefined symbol: __nvJitLinkAddData_12_1, version libnvJitLink.so.12
    ```

    4. 将onnx文件转为engine文件
    ```
    cd src/main/detection/models
    # 请将下面的路径替换为您的TensorRT实际安装路径
    /path/to/your/TensorRT-8.x.x.x/bin/trtexec --onnx=ignore/car_12s.onnx --saveEngine=ignore/car_12s.engine --fp16 

    # 以及装甲板的模型
    /path/to/your/TensorRT-8.x.x.x/bin/trtexec --onnx=ignore/armor_11s.onnx --saveEngine=ignore/armor_11s.engine --fp16 
    ```
## 硬件

## 运行
以下为执行节点的命令，使用前请自行输入```source install/setup.bash```。另外，建议日常调试时写sh脚本，例如打开一个终端并执行相机节点
```
gnome-terminal -- bash -c "cd /your/path/to/this/folder; source install/setup.bash; ros2 run camera image_creator; exec bash"
```

### 主流程
```
ros2 run camera image_creator                           # 相机
ros2 run detection detection                            # 识别

ros2 launch livox_ros2_driver livox_lidar_launch.py     # 激光雷达

ros2 run filter filter                                  # 过滤
ros2 run cluster cluster                                # 聚类

ros2 run relocalization icp                             # 重定位

ros2 run locate locate                                  # 融合

ros2 run sp_referee main                                # 与裁判系统通信
ros2 run decision decision                              # 决策双倍易伤
```

### 扫描建图
如果需要在比赛之外的场地调试，而且没有哨兵扫的图，使用如下两个命令，并将```filter/config/filter.yaml```的```use_static_scan```设置为```true```
```
ros2 run filter static_scan                             # 扫描建图
ros2 run filter pcd_process                             # 体素化
```


### 调试
```
ros2 run minimap minimap_drawer                         # 小地图，显示定位结果
ros2 run relocalization viewer                          # 重定位结果的可视化
ros2 run image_viewer image_viewer                      # 查看ros话题中的图像
```

## 开发中的技巧
### vscode的红色波浪线
哪个头文件报错，就把鼠标放在对应的红色波浪线上，点击“Quick Fix”，后续分状况
1. c++标准库报错：选中“Edit "includePath" setting”，点击“Compiler path”，选择g++
2. opencv头文件报错：选中第一行“Add "includePath":/usr/include/opencv4”
3. ros相关头文件表错：点击“Quick Fix”，选中“Edit "includePath" setting”，在“Include path”下添加“/opt/ros/humble/include/**”