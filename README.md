# sp_radar_26使用指南
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

