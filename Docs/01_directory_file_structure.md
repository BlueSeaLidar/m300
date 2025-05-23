# 1 Directory file structure

## 目录说明

+ `M300-ROS`  ROS driver
+ `M300-ROS2` ROS2 driver
+ `M300-SDK`  SDK driver

note:only the demo is different between the three, the underlying structure is identical.

## SDK struct

```C++
├── samples                     
│   ├──CommandControl.cpp
│   ├──PointCloudAndImu.cpp
│   └──Upgrade.cpp
├── sdk
│   ├── define.h
│   ├── protocol.h
│   ├── global.h
│   ├── global.cpp
├   ├── pacecatlidarsdk.h
│   └── pacecatlidarsdk.cpp
├── CMakeLists.txt
```

## ROS结构

```C++
├── launch
│   └──LDS-M300_E.launch
├── msg
│   ├──CustomPoint.msg
│   └──CustomMsg.msg
├── rviz
│   └──1.rviz
├── src
│   └──node.cpp
├── sdk
│   ├── define.h
│   ├── protocol.h
│   ├── global.h
│   ├── global.cpp
│   ├── pacecatlidarsdk.h
│   └── pacecatlidarsdk.cpp
├── CMakeLists.txt
├── package.xml
```

## ROS2结构

```C++
|                       ├── launch
|                       │   └──LDS-M300_E.launch
|                       ├── param
|                       │   └──LDS-M300-E.yaml
|                       ├── include
├── pacecat_m300_driver ├── src
│                       │   ├──client.cpp
|                       |   └──driver.cpp
│                       ├── sdk
│                       │   ├── define.h
│                       │   ├── protocol.h
│                       │   ├── global.h
│                       │   ├── global.cpp
│                       │   ├── pacecatlidarsdk.h
│                       │   └── pacecatlidarsdk.cpp
│                       ├── CMakeLists.txt
│                       ├── package.xml
│
│
│                      ├── include
│                      ├── msg
├──pacecat_m300_inter  │   ├── CustomMsg.msg
|                      │   └── CustomPoint.msg
|                      ├── src
|                      ├── srv
|                          └──Control.srv
```
