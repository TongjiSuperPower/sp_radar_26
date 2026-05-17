# sp_radar_26 User Guide  
## Compilation  
- System environment: Ubuntu 22.04, **do not use Vmware**  
- One-click installation of ROS2 using FishROS  

    ```wget http://fishros.com/install -O fishros && . fishros```  

- NVIDIA driver (>=525), [CUDA Toolkit (12.0) (runfile installation)](https://developer.nvidia.com/cuda-12-0-0-download-archive?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=runfile_local), [cudnn (8.9.7) (deb installation)](https://developer.nvidia.com/rdp/cudnn-archive), [tensorrt (8.6.1) (tar installation)](https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/secure/8.6.1/tars/TensorRT-8.6.1.6.Linux.x86_64-gnu.cuda-12.0.tar.gz)  

    When installing CUDA Toolkit, you can choose to install the driver, i.e., you can start directly with the runfile installation of CUDA Toolkit. Of course, you can also download the driver via Ubuntu's apt or software & updates.  

- The following libraries also need to be installed during compilation:  

    - [serial](https://blog.csdn.net/weixin_42670590/article/details/137887249)  
    ```
    mkdir serial
    git clone https://github.com/ZhaoXiangBox/serial
    or (git clone https://gitee.com/laiguanren/serial.git)
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

    - [Open3D](https://blog.csdn.net/m0_51661400/article/details/134735883) [(ZSTD library dependency error during Open3D installation)](https://blog.csdn.net/m0_51661400/article/details/134735883)  

    ```
    git clone --recursive https://github.com/intel-isl/Open3D
    cd Open3D
    bash util/install_deps_ubuntu.sh

    cd Open3D
    mkdir build
    cd build
    cmake ..
    # You can also use the following command according to your needs
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

- For the first compilation, first compile the `radar_msgs` package, `livox_sdk_vendor`, and `livox_interfaces`. After compilation, remember to run  
```source install/setup.bash```  

- When compiling the `detection` package, you need to add the `--symlink` parameter. If you encounter missing dynamic link libraries (e.g., `libdeploy.so`, `libMvControl.so`) during runtime, adding this parameter will resolve the issue.  

## YOLO Model Format Preparation  
- The recognition module requires YOLO engine files during runtime. The repository provides the corresponding `.pt` files, which need to be converted as follows:  

    1. Install conda  
    ```
    wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
    bash Miniconda3-latest-Linux-x86_64.sh
    ```

    2. Create and configure a virtual environment. For details, refer to [trtyolo-export](https://github.com/laugh12321/TensorRT-YOLO/tree/export)  
    ```
    conda create --name trtyolo python=3.8
    conda activate trtyolo
    pip install trtyolo-export
    ```

    3. Convert the `.pt` file to an ONNX file  
    ```
    cd src/main/detection/models
    trtyolo export -w car_12s.pt -v ultralytics -o ignore --max_boxes 100 --iou_thres 0.45 --conf_thres 0.25    
    
    # Armor model
    trtyolo export -w armor_11s.pt -v ultralytics -o ignore --max_boxes 100 --iou_thres 0.45 --conf_thres 0.25   
    ```

    - Note: If you encounter an error similar to the one below, refer to this [blog](https://blog.csdn.net/qq_42730750/article/details/139582293)  
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

    4. Convert the ONNX file to an engine file  
    ```
    cd src/main/detection/models
    # Please replace the path below with your actual TensorRT installation path
    /path/to/your/TensorRT-8.x.x.x/bin/trtexec --onnx=ignore/car_12s.onnx --saveEngine=ignore/car_12s.engine --fp16 

    # Armor model
    /path/to/your/TensorRT-8.x.x.x/bin/trtexec --onnx=ignore/armor_11s.onnx --saveEngine=ignore/armor_11s.engine --fp16 
    ```

## Hardware  

## Running  
Below are the commands to execute nodes. Before using them, make sure to run `source install/setup.bash`. Additionally, it is recommended to write shell scripts for daily debugging. For example, to open a terminal and run the camera node:  
```
gnome-terminal -- bash -c "cd /your/path/to/this/folder; source install/setup.bash; ros2 run camera image_creator; exec bash"
```

### Main Workflow  
```
ros2 run camera image_creator                           # Camera  
ros2 run detection detection                            # Detection  

ros2 launch livox_ros2_driver livox_lidar_launch.py     # LiDAR  

ros2 run filter filter                                  # Filtering  
ros2 run cluster cluster                                # Clustering  

ros2 run relocalization icp                             # Relocalization  

ros2 run locate locate                                  # Fusion  

ros2 run sp_referee main                                # Communication with the referee system  
ros2 run decision decision                              # Decision-making for double damage  
```

### Scanning and Mapping  
If you need to debug in a location other than the competition venue and do not have a map scanned by the sentry, use the following two commands and set `use_static_scan` to `true` in `filter/config/filter.yaml`.  
```
ros2 run filter static_scan                             # Scanning and mapping  
ros2 run filter pcd_process                             # Voxelization  
```

### Debugging  
```
ros2 run minimap minimap_drawer                         # Mini-map, displays localization results  
ros2 run relocalization viewer                          # Visualization of relocalization results  
ros2 run image_viewer image_viewer                      # View images from ROS topics  
```

## Development Tips  
### Red Squiggly Lines in VSCode  
For any header file that shows an error, hover over the red squiggly line, click "Quick Fix," and proceed based on the situation:  
1. C++ standard library error: Select "Edit 'includePath' setting," click "Compiler path," and choose g++.  
2. OpenCV header file error: Select the first option, "Add 'includePath':/usr/include/opencv4."  
3. ROS-related header file error: Click "Quick Fix," select "Edit 'includePath' setting," and under "Include path," add `/opt/ros/humble/include/**`.