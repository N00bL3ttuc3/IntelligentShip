# Intelligent Ship #

---

## Introduction ##


This ship has many functions such that

- target detection
- target tracking 
- high-precision positioning, 
- simultaneous localization and mapping(SLAM) 
- multiple control methods
- autonomous cruise
- autonomous obstacle avoidance
- autonomous shooting

## Architecture ##

```sh
├── CMakeLists.txt -> /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake
├── control
│   ├── CMakeLists.txt
│   ├── launch
│   │   ├── all.launch
│   │   ├── all_t.launch
│   │   ├── attack.launch
│   │   ├── convert.launch
│   │   ├── rec.launch
│   │   ├── record.launch
│   │   ├── stanley.launch
│   │   ├── stanley_t.launch
│   │   ├── testrec.launch
│   │   └── video.launch
│   ├── msg
│   │   └── Command.msg
│   ├── package.xml
│   ├── readme.md
│   └── script
│       ├── attack.py
│       ├── autorecord.py
│       ├── ccc.py
│       ├── convert.py
│       ├── cubic_spline.py
│       ├── lqr_control_node.py
│       ├── __pycache__
│       ├── rec_control.py
│       ├── stanley_control (copy).py
│       ├── stanley_control.py
│       ├── stanley_control_t.py
│       ├── ttt.py
│       └── video_control.py
├── getyaw
│   ├── cmake-build-debug
│   │   ├── atomic_configure
│   │   ├── bin
│   │   ├── catkin
│   │   ├── catkin_generated
│   │   ├── CATKIN_IGNORE
│   │   ├── CMakeCache.txt
│   │   ├── CMakeFiles
│   │   ├── cmake_install.cmake
│   │   ├── CTestConfiguration.ini
│   │   ├── CTestCustom.cmake
│   │   ├── CTestTestfile.cmake
│   │   ├── devel
│   │   ├── getyaw.cbp
│   │   ├── gtest
│   │   ├── lib
│   │   ├── Makefile
│   │   └── test_results
│   ├── CMakeLists.txt
│   ├── include
│   │   └── getyaw
│   ├── launch
│   │   └── getyaw.launch
│   ├── package.xml
│   └── src
│       └── getyaw_node.cpp
├── LICENSE
├── read_cam
│   ├── CMakeLists.txt
│   ├── include
│   │   └── read_cam
│   ├── launch
│   │   └── cam.launch
│   ├── package.xml
│   └── scripts
│       └── read_cam.py
├── README.md
├── rostostm32
│   ├── CMakeLists.txt
│   ├── include
│   │   └── rostostm32
│   ├── launch
│   │   └── ros2stm32.launch
│   ├── package.xml
│   └── src
│       ├── rostostm32
│       └── rostostm32_node.cpp
├── slamware_ros_sdk
│   ├── cfg
│   │   └── diff_drive
│   ├── CMakeLists.txt
│   ├── include
│   │   └── slamware_ros_sdk
│   ├── launch
│   │   ├── slamware_ros_sdk_intergated_with_teb.launch
│   │   ├── slamware_ros_sdk_server_and_view.launch
│   │   ├── slamware_ros_sdk_server_node_datapub.launch
│   │   ├── slamware_ros_sdk_server_node.launch
│   │   ├── view_slamware_ros_sdk_server_node_datapub.launch
│   │   └── view_slamware_ros_sdk_server_node.launch
│   ├── LICENSE
│   ├── msg
│   │   ├── ActionDirection.msg
│   │   ├── AddLineRequest.msg
│   │   ├── AddLinesRequest.msg
│   │   ├── ArtifactUsage.msg
│   │   ├── BasicSensorInfoArray.msg
│   │   ├── BasicSensorInfo.msg
│   │   ├── BasicSensorValueDataArray.msg
│   │   ├── BasicSensorValueData.msg
│   │   ├── BasicSensorValue.msg
│   │   ├── CancelActionRequest.msg
│   │   ├── ClearLinesRequest.msg
│   │   ├── ClearMapRequest.msg
│   │   ├── GoHomeRequest.msg
│   │   ├── ImpactType.msg
│   │   ├── Line2DFlt32Array.msg
│   │   ├── Line2DFlt32.msg
│   │   ├── LocalizationMovement.msg
│   │   ├── LocalizationOptions.msg
│   │   ├── MapKind.msg
│   │   ├── MoveByDirectionRequest.msg
│   │   ├── MoveByThetaRequest.msg
│   │   ├── MoveLineRequest.msg
│   │   ├── MoveLinesRequest.msg
│   │   ├── MoveOptionFlag.msg
│   │   ├── MoveOptions.msg
│   │   ├── MoveToLocationsRequest.msg
│   │   ├── MoveToRequest.msg
│   │   ├── OptionalBool.msg
│   │   ├── OptionalFlt32.msg
│   │   ├── OptionalFlt64.msg
│   │   ├── OptionalInt16.msg
│   │   ├── OptionalInt32.msg
│   │   ├── OptionalInt64.msg
│   │   ├── OptionalInt8.msg
│   │   ├── OptionalLocalizationMovement.msg
│   │   ├── OptionalUInt16.msg
│   │   ├── OptionalUInt32.msg
│   │   ├── OptionalUInt64.msg
│   │   ├── OptionalUInt8.msg
│   │   ├── RecoverLocalizationRequest.msg
│   │   ├── RectFlt32.msg
│   │   ├── RectInt32.msg
│   │   ├── RemoveLineRequest.msg
│   │   ├── RobotBasicState.msg
│   │   ├── RobotDeviceInfo.msg
│   │   ├── RotateRequest.msg
│   │   ├── RotateToRequest.msg
│   │   ├── SensorType.msg
│   │   ├── SetMapLocalizationRequest.msg
│   │   ├── SetMapUpdateRequest.msg
│   │   ├── SyncMapRequest.msg
│   │   ├── Vec2DFlt32.msg
│   │   └── Vec2DInt32.msg
│   ├── package.xml
│   ├── README.md
│   ├── rviz
│   │   ├── slamware_ros_sdk_server_node_datapub.rviz
│   │   ├── slamware_ros_sdk_server_node.rviz
│   │   └── slamware_ros_sdk_teb.rviz
│   ├── src
│   │   ├── client
│   │   └── server
│   └── srv
│       ├── SyncGetStcm.srv
│       └── SyncSetStcm.srv
├── slamware_sdk
│   ├── CMakeLists.txt
│   ├── include
│   │   ├── boost
│   │   ├── Eigen
│   │   ├── json
│   │   └── rpos
│   ├── lib
│   │   └── linux-x86_64-gcc5.4
│   ├── LICENSE
│   └── package.xml
├── usb_cam
│   ├── AUTHORS.md
│   ├── CHANGELOG.rst
│   ├── CMakeLists.txt
│   ├── include
│   │   └── usb_cam
│   ├── launch
│   │   └── usb_cam-test.launch
│   ├── LICENSE
│   ├── mainpage.dox
│   ├── nodes
│   │   └── usb_cam_node.cpp
│   ├── package.xml
│   ├── README.md
│   └── src
│       ├── LICENSE
│       └── usb_cam.cpp
└── vis_simulator
    ├── CMakeLists.txt
    ├── package.xml
    └── script
        └── boat_simulator.py



```


