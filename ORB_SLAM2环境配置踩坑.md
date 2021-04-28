#### 搭建环境：Ubuntu16.04+ROS

#### 一. 安装相关工具

1. 更新apt

   ```
   sudo apt-get update
   ```

2. 安装git

   ```
   sudo apt-get install git
   ```

3. 安装cmake

   ```
   sudo apt-get install cmake
   ```

4. 安装gcc、g++

   ```
   sudo apt-get install g++ 
   sudo apt-get install gcc  
   ```

#### 二. 安装Pangolin

1. 安装依赖项

   ```
   sudo apt-get install libglew-dev
   sudo apt-get install libboost-dev libboost-thread-dev libboost-filesystem-dev
   sudo apt-get install libpython2.7-dev
   ```

2. 下载Pangolin

   ```
   git clone https://github.com/stevenlovegrove/Pangolin.git
   ```

3. 安装

   ```
   cd Pangolin
   mkdir build
   cd build
   cmake -DCPP11_NO_BOOSR=1 ..
   make -j8
   sudo make install
   ```

#### 三. 安装opencv

1. 安装依赖项

   ```
   sudo apt-get install build-essential
   sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
   sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
   ```

2. 下载opencv并解压

   ```
   wget -O opencv-3.4.1.zip https://github.com/Itseez/opencv/archive/3.4.1.zip
   unzip opencv-3.4.1.zip
   ```

3. 编译并安装

   ```
   cd ~/opencv-3.4.1
   mkdir build
   cd build
   cmake -D CMAKE_BUILD_TYPE=Release –D CMAKE_INSTALL_PREFIX=/usr/local ..
   make -j8
   sudo make install
   ```

4. 将opencv的库加入路径并使之生效

   ```
   sudo gedit /etc/ld.so.conf.d/opencv.conf 
   添加：/usr/local/lib
   保存关闭后：sudo ldconfig  
   ```

5. 配置bash

   ```
   sudo gedit /etc/bash.bashrc  
   添加：PKG_CONFIG_PATH=$PKG_CONFIG_PATH=/usr/local/lib/pkgconfig
   保存关闭后：source /etc/bash.bashrc 
   ```

#### 四. 安装Eigen3（开源线性，用于矩阵计算）

 1. ```
    sudo apt-get install libeigen3-dev
    ```

#### 五. 安装ORB_SLAM2

1. 检查环境（此处为网上大多教程未写，即我们下载的ORB_SLAM2是ROS的一个功能包，无法单独使用，需要在一个工作空间下编译运行均，因此如果没有创建工作空间，首先执行以下步骤）

   ```
   mkdir catkin_ws & cd catkin_ws
   mkdir src & cd src
   catkin_init_workspace
   cd .. & catkin_mak
   cd catkin_ws/src
   ```

2. 下载ORB_SLAM2

   ```
   git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
   ```

3. 编译运行build.sh

   ```
   cd ORB_SLAM2
   chmod +x build.sh
   ./build.sh
   ```

4. 配置环境

   ```
   sudo gedit ~/.bashrc
   添加：
   	source ./catkin_ws/devel/setup.sh
   	export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:catkin_ws所在路径/src/ORB_SLAM2/Examples/ROS
   保存退出后：source ~/.bashrc
   ```

5. 编译运行build_ros.sh

   ```
   chmod +x build_ros.sh
   ./build_ros.sh
   ```

6. 错误处理

   运行./build_ros.sh时，可能会保错，若是因为找不到libboost_system.so、libboost_system.so.1.58.0、libboost_filesystem.so和libboost_filesystem.so.1.58.0，则进行以下操作

   ```
   locate boost_system
   locate boost_filesystem
   ```

   将对应搜索到的.so库均复制到ORB_SLAM2/lib下

   ```
   cd ORB_SLAM2/Examples/ROS/ORB_SLAM2
   sudo gedit ./Cmakelists.txt
   ```

   找到set()函数，添加以下两行

   ```
   ${PROJECT_SOURCE_DIR}/../../../lib/libboost_filesystem.so
   ${PROJECT_SOURCE_DIR}/../../../lib/libboost_system.so
   ```

   再次返回ORB_SLAM2目录，运行./build_ros.sh即可

#### 六. 安装usb_cam（使用自己电脑的摄像头进行测试）

1. 下载usb_cam

   ```
   cd catkin_ws/src
   git clone https://github.com/bosch-ros-pkg/usb_cam.git
   ```

2. 编译工作空间

   ```
   cd ..
   ctakin_make
   ```

3. 编译安装usb_cam功能包

   ```
   cd src/usb_cam
   mkdir build
   cd build
   cmake ..
   make -j8
   ```

4. 修改权限

   将usb_cam-test.launch勾选允许作为程序运行文件选项

5. 修改相机节点

   ```
   cd ORB_SLAM2/Examples/ROS/ORB_SLAM2/src
   sudo gedit ros_mono.cc
   找到创建订阅者语句，将话题改为usb_cam/image_raw
   ```

6. 再次编译build_ros.sh

   ```
   cd ORB_SLAM2
   ./build_ros.sh
   ```

七. 测试运行

 1. 打开摄像头

    ```
    roslaunch usb_cam usb_cam-test.launch
    ```

 2. 启动ORB_SLAM2

    ```
    rosrun ORB_SLAM2 Mono xxxxxx/catkin_ws/src/ORB_SLAM2/Vocabulary/ORBvoc.txt xxxxxx/catkin_ws/src/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Asus.yaml
    ```

    