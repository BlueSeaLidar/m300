# M300  driver

---

## SDK

windows/linux SDK and Demo programs for  LDS-M300-E lidar

### HOW TO BUILD AND USE

***EXE file explain:***

>PointCloudAndImu:    Output point cloud data of each frame of lidar and real-time imu data
>CommandControl:      Output lidar supported commands and return values
>Upgrade:             lidar Firmware Upgrade (motor)

### LINUX

***Prerequisite: g++ and gcc must be installed***

    cmake CMakeList.txt
    make
    ./${EXE file}            EXE file{PointCloudAndImu,CommandControl,Upgrade}

### WINDOWS

use cmake_gui  build ,open project and compile with visual stdioxx,then enerate a solution

---

## ROS

PACECAT ROS driver is specially designed to connect to the lidar products produced by our company. The driver can run on operating systems with ROS installed, and mainly supports ubuntu series operating systems (14.04LTS-20.04LTS). The hardware platforms that have been tested to run the ROS driver include: Intel x86 mainstream CPU platform, and some ARM64 hardware platforms (such as NVIDIA, Rockchip, Raspberry Pi, etc., which may need to update the cp210x driver).

***Please ensure that the path does not contain Chinese characters, otherwise the compilation will fail!***

### Get  Build  Run ROS

    git clone https://github.com/BlueSeaLidar/m300.git 
    cd m300/M300-ROS
    catkin_make
    source devel/setup.sh
    roslaunch pacecat_m300 LDS-M300_E.launch

---

## ROS2

PACECAT ROS2 driver is specially designed to connect to the lidar products produced by our company. The driver can run on operating systems with ROS2 installed, and mainly supports ubuntu series operating systems (18.04LTS-now). The hardware platforms that have been tested to run the ROS2 driver include: Intel x86 mainstream CPU platform, and some ARM64 hardware platforms (such as NVIDIA, Rockchip, Raspberry Pi, etc., which may need to update the cp210x driver).

***Please ensure that the path does not contain Chinese characters, otherwise the compilation will fail!***

### Get  Build  Run ROS2

    git clone https://github.com/BlueSeaLidar/m300.git 
    cd m300/M300-ROS2
    colcon build
    source install/setup.sh 
    ros2 launch pacecat_m300_driver  LDS-M300-E.launch

## Driver launch launch file

>frame_id : Name of the flag coordinate system
>topic_pointcloud : pointcloud  topic print
>output_pointcloud: pointcloud  topic print enable
>topic_custommsg  : custom msg topic print
>output_custommsg : custom msg topic print enable
>topic_imu        : imu topic
>output_imu       :imu topic enable
>lidar_ip         :lidar ip
>lidar_port       :lidar port
>local_port       :listen port
>ptp_enable       :ptp enable  -1 not set 0 off 1 on
>timemode         :timestamp format    0:lidar start up time  1：system timestamp(default)
>
>#shadows filter param
>sfp_enable         :enable/unable
>window             :check windows size
>min_angle          :min angle
>max_angle          :max angle
>effective_distance :compare distance
>
>#dirty filter param
>dfp_enable:        enable/unable
>continuous_times:  continuous frame rate
>dirty_factor:      threshold
>
>#matrix rotate  roll/pitch/yaw: angle value(not radian value)  x/y/z:distance unit  m
>mr_enable:
>roll
>pitch
>yaw
>x
>y
>z

### For detailed documentation, please refer to the following

[目录文件结构](./Docs/01_目录文件结构.md)|[directory file structure](./Docs/01_directory_file_structure.md)

[线程模型](./Docs/02_线程模型.md)|[thread model](./Docs/02_thread_model.md)

[接口说明](./Docs/03_接口说明.md)|[interface_description](./Docs/03_interface_description.md)
