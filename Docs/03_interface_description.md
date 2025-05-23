# 3 Interface description

This section focuses on the interfaces called externally in the PaceCatLidarSDK class.

***#include<pacecatlidarsdk.h>***

## Enumeration

```C++
typedef enum {
    LIDARIMUDATA = 0,
    LIDARPOINTCLOUD = 0x01,
}LidarPointDataType;
```

```C++
enum LidarState
{
    OFFLINE = 0,
    ONLINE,
    QUIT
};
```

```C++
enum LidarAction
{
    NONE,
    FINISH,
    START,
    STOP,
    GET_PARAMS,
    GET_VERSION,
    SET_NETWORK,
    SET_UPLOAD_NETWORK,
    SET_UPLOAD_FIX,
    UPGRADE
};
```


## 结构体

```C++

typedef struct {
    uint8_t version;            //
    uint32_t length;            //
    uint16_t time_interval;     //unit: 0.1 us
    uint16_t dot_num;           //one packet  points num
    uint16_t udp_cnt;           //packet counting
    uint8_t frame_cnt;          //frame counting
    uint8_t data_type;          //data type,refer enum LidarPointDataType
    uint8_t time_type;          //not use
    uint8_t rsvd[12];           //reserved
    uint32_t crc32;             //CRC
    uint64_t timestamp;         //unit: ns
    uint8_t data[0];            //data point
} LidarPacketData;              //packet head

```

```C++
typedef struct {
    uint32_t offset_time;       //offset time relative to the base time
    float x;                    // X axis, unit:m
    float y;                    // Y axis, unit:m
    float z;                    // Z axis, unit:m
    uint8_t reflectivity;       // reflectivity, 0~255
    uint8_t tag;                // not use
    uint8_t line;               // not use
} LidarCloudPointData;          // one point data struct
```

```C++
typedef struct {
    float gyro_x;               //unit:rad/s
    float gyro_y;               //unit:rad/s
    float gyro_z;               //unit:rad/s
    float acc_x;                //unit:g
    float acc_y;                //unit:g
    float acc_z;                //unit:g
    float linear_acceleration_x;//not use
    float linear_acceleration_y;//not use
    float linear_acceleration_z;//not use
} LidarImuPointData;            //imu packet struct
```

```C++
typedef struct
{
    char lidar_ip[16];          //IP
    int lidar_port;             //port
    int listen_port;            //local listen port
    int ptp_enable;             //PTP flag
    int frame_package_num;      //one frame packet num
    int timemode;               //timestamp mode：0 lidar time 1 local time
} ArgData;                      //lidar param
```

```C++
typedef struct
{
    int sfp_enable;             // enable or unable
    int window;                 //check windows size
    double min_angle;           //min angle
    double max_angle;           //max angle
    double effective_distance;  //compare distance
}ShadowsFilterParam;            //outlier-removed param
```

```C++
typedef struct
{
    int dfp_enable;             //enable or unable
    int continuous_times;       //continuous frame rate
    double dirty_factor;        //threshold
}DirtyFilterParam;              //
```

```C++
typedef struct
{
    int mr_enable;              //enable or unable
    double trans[3];            //translation data
    double rotation[3][3];      //rotate data
}MatrixRotate_2;                //
```

```C++
typedef struct
{
    std::string uuid;           //SN
    std::string model;          //model
    std::string lidarip;        //IP
    std::string lidarmask;      //masek
    std::string lidargateway;   //gateway
    uint16_t lidarport;         //port
    std::string uploadip;       //upload ip
    uint16_t uploadport;        //upload port
    uint8_t uploadfix;          //upload  fix ip/port
}BaseInfo;                      //
```

```C++
typedef struct
{
    std::string mcu_ver;        //
    std::string motor_ver;      //
    std::string software_ver;   //
}VersionInfo;                   //
```

```C++
struct RunConfig
{
    int ID;                                         //lidar ID
    std::thread  thread_subData;                    //work thread
    LidarCloudPointCallback  cb_cloudpoint;         //Point cloud callback pointer set by the user
    void *cloudpoint;                               //Point cloud global data passed by the user
    LidarImuDataCallback cb_imudata;                //IMU callback pointer set by the user
    void *imudata;                                  //IMU global data passed by the user
    LidarLogDataCallback cb_logdata;                //Log callback pointer set by the user
    void*logdata;                                   //Log global data passed by the user
    int run_state;                                  //run state
    std::string lidar_ip;                           //lidar ip
    int lidar_port;                                 //lidar port
    int listen_port;                                //local listen port
    std::vector<LidarCloudPointData> cloud_data;    //save pointcloud packet data
    std::queue<IIM42652_FIFO_PACKET_16_ST> imu_data;//imu data queue
    IMUDrift  imu_drift;                            //imu calibration data
    uint32_t frame_cnt;                             //frame  rate
    uint64_t frame_firstpoint_timestamp;            //first packet timestamp from every frame      
    int action;                                     //ctrl cmd
    int send_len;                                   //send cmd len
    std::string send_buf;                           //send cmd data
    int recv_len;                                   //recv cmd len
    std::string recv_buf;                           //recv cmd data
    int ptp_enable;                                 //PTP enable
    ShadowsFilterParam sfp ;                        //outlier-removed  param
    DirtyFilterParam   dfp;                         //dirty filter param
    MatrixRotate_2 mr_2;                            //matrix  rotate param, control  pointcloud adjust position
    int frame_package_num;                          //set packet num  in one frame
    int timemode;                                   //time mode
};
```

## 函数指针

```C++
@Name    LidarCloudPointCallback
@Brief   set function pointers for pointcloud data
@Param   uint32_t handle                 lidar ID
@Param   const uint8_t dev_type          data type
@Param   const LidarPacketData *data     user Level data
@Param   void* client_data               pointer to data that the user needs to pass globally
@Return  pointer
@Note    user thread input
```

```C++
@Name    LidarImuDataCallback
@Brief   set function pointers for imu data
@Param   uint32_t handle             lidar ID
@Param   const uint8_t dev_type      data type
@Param   const LidarPacketData *data user level data
@Param   void* client_data           pointer to data that the user needs to pass globally
@Return  pointer
@Note    user thread input
```

```C++
@Name    LidarLogDataCallback
@Brief   set function pointers for log data
@Param   uint32_t handle             lidar ID
@Param   const uint8_t dev_type      data type
@Param   const char *data            user level data
@Param   int len                     user level len
@Return  pointer
@Note    user thread input
```

## 函数

```C++
@Name    Init
@Brief   Class initialization, mainly to enable heartbeat packet threads
@Return  void
@Note    The first step in running the example

```

```C++
@Name    Uninit
@Brief   Class counter-initialization, mainly to close all worker threads
@Return  void
@Note    NULL

```

```C++
@Name    SetPointCloudCallback
@Brief   Setting up callback functions for point cloud data
@Param   int ID                      lidar ID
@Param   LidarCloudPointCallback cb  Pointer to the callback function
@Param   void* client_data           Data pointers that the user needs to use globally
@Return  bool
@Note    
```

```C++
@Name    SetImuDataCallback
@Brief   Setting up callback functions for imu
@Param   int ID                      lidar ID
@Param   LidarImuDataCallback cb     Pointer to the callback function
@Param   void* client_data           Data pointers that the user needs to use globally
@Return  bool
@Note    
```

```C++
@Name    SetLogDataCallback
@Brief   Setting up callback functions for log
@Param   int ID                      lidar ID
@Param   LidarImuDataCallback cb     Pointer to the callback function
@Param   void* client_data           Data pointers that the user needs to use globally
@Return  bool
@Note    
```

```C++
@Name    WritePointCloud
@Brief   write pointcloud data
@Param   int ID                      
@Param   const uint8_t dev_type      
@Param   LidarPacketData *data       
@Return  void
@Note    inside use
```

```C++
@Name    WriteImuData
@Brief   write imu data
@Param   int ID                      
@Param   const uint8_t dev_type      
@Param   LidarPacketData *data       
@Return  void
@Note    inside use
```

```C++
@Name    WriteLogData
@Brief   write log data
@Param   int ID                      
@Param   const uint8_t dev_type      
@Param   char* *data                 
@Param   int len                     
@Return  void
@Note    inside use
```

```C++
@Name    AddLidar
@Brief   add lidar
@Param   ArgData argdata             
@Param   ShadowsFilterParam sfp      outlier removed param
@Param   DirtyFilterParam dfp        dirty filter param
@Param   MatrixRotate_2 mr_2         matrix rotate param
@Return  int                         lidar ID
@Note     
```

```C++
@Name    ConnectLidar
@Brief   Connects to lidar, sends commands, receives data.
@Param   int ID                      lidar ID
@Return  bool
@Note   
```

```C++
@Name    DisconnectLidar
@Brief   disconnect lidar
@Param   int ID                      lidar ID
@Return  bool
@Note   
```

```C++
@Name    QueryBaseInfo
@Brief   query lidar base info
@Param   int ID                      lidar ID
@Param   BaseInfo info               
@Return  bool
@Note   
```

```C++
@Name    QueryVersion
@Brief   query lidar version
@Param   int ID                      lidar ID
@Param   VersionInfo info            
@Return  bool
@Note   
```

```C++
@Name    QueryDeviceState
@Brief   query device state
@Param   int ID                      lidar ID
@Return  int LidarState
@Note   
```

```C++
@Name    SetLidarNetWork
@Brief   set lidar network
@Param   int ID                      lidar ID
@Param   std::string ip              ip
@Param   std::string mask            mask
@Param   std::string gateway         gateway
@Param   uint16_t port               port
@Return  bool                        
@Note   
```

```C++
@Name    SetLidarUploadNetWork
@Brief   set lidar upload network
@Param   int ID                      lidar ID
@Param   std::string upload_ip       upload ip
@Param   uint16_t upload_port        upload port
@Return  bool                        
@Note   
```

```C++
@Name    SetLidarUploadFix
@Brief   set lidar upload fix  ip/port
@Param   int ID                      lidar ID
@Param   bool isfix                  
@Return  bool                        
@Note   
```

```C++
@Name    SetLidarAction
@Brief   Setting lidar control parameters
@Param   int ID                      lidar ID
@Param   int action                  refer LidarAction
@Return  bool                        
@Note   
```

```C++
@Name    SetLidarUpgrade
@Brief   set lidar upgrade
@Param   int ID                      lidar ID
@Param   std::string path            Firmware Path
@Return  bool                        
@Note   
```

```C++
@Name    read_calib
@Brief   Read calibration parameters
@Param   const char* lidar_ip         lidar ip
@Param   int port                     lidar port
@Return  int                        
@Note   
```

```C++
@Name  QueryIDByIp
@Brief   query id by lidar ip
@Param   const char* lidar_ip         lidar ip
@Return   int                         lidar ID                  
@Note   
```

```C++
@Name    FirmwareUpgrade
@Brief   Firmware upgrade
@Param   const char* lidar_ip         lidar ip
@Param   int port                     lidar port
@Param   std::string path             Firmware Path
@Param   std::string& error           error msg
@Return  bool                                          
@Note    inside use
```

```C++
@Name    GetConfig
@Brief   Get configuration information based on lidar ID
@Param   int ID                       lidar ID
@Return  RunConfig                   lidar config                       
@Note   
```
