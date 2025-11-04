#ifndef M300_E_PROTOCOL_H
#define M300_E_PROTOCOL_H

#include "define.h"

#define TRANS_BLOCK 0x200
#define PACK_PREAMLE 0X484C
#define DRIFT_RD_PACK 0x4357
#define DRIFT_MAGIC 0xD81F1CA1
#define CMD_REPEAT 50
#define LOCALPORT 6668
#define HEARTPORT 6789
#define PAGE_SIZE 256
#define LINE_SIZE 64

#define GS_PACK 0x4753
#define S_PACK 0x0053
#define C_PACK 0x0043
#define RG_PACK 0x5247
#define F_PACK 0x0046

#define OP_FLASH_ERASE 0xFE00EEEE
#define OP_WRITE_IAP 0xFE00AAAA
#define OP_FRIMWARE_RESET 0xFE00BBBB

#define PACK_PREAMLE 0X484C
#define F_PACK 0x0046
#define TIMEOUT 3
#define BUFFERSIZE 1024

#define UDP_HEADER              8

#define TAG_MIRROR_NOT_STABLE 0x80
#define TAG_MOTOR_NOT_STABLE 0x40
#define TAG_DUAL_ECHO_MODE   0x20
#define TAG_WITH_RAIN_DETECT  0x10

#define ONE_FRAME_BUFFER_SIZE 300 * 20 * 128
// debug   factor
#define ZERO_POINT_FACTOR  0.8	   // 零点的比例
#define DISTANCE_CLOSE_FACTOR  0.8 // 0.2m以内
#define SUM_POINT_NUM_FACTOR  0.2  // 总点数占比
	
#define POINTCLOUD_TIMESTAMP_MAX_DIFF  850000//点云包的时间间隔最大值  标准系数设置为5
#define IMU_TIMESTAMP_MAX_DIFF 7500000//imu包的时间间隔最大值
#define POINTCLOUD_IMU_TIMESTAMP_MAX_DIFF 10000000//点云包和imu包的最大时间间隔

typedef struct
{
	uint16_t code;
	uint16_t len;
	uint16_t idx;
	uint16_t pad;
	uint8_t data[TRANS_BLOCK];
} TransBuf;

struct CmdHeader
{
	uint16_t sign;
	uint16_t cmd;
	uint16_t sn;
	uint16_t len;
};

typedef struct
{
	double R[3][3];
	float K[3];
	float B[3];
	double Gyro[3];
} IMUDrift;

typedef struct
{
	IMUDrift imu;
} Drifts;

typedef struct
{
	uint32_t code;
	union
	{
		Drifts drifts;
		uint32_t body[126];
	};
	uint32_t crc;
} DriftCalib;

#pragma pack(push, 1)

typedef struct
{
	uint8_t Header;
	int16_t Accel_X;
	int16_t Accel_Y;
	int16_t Accel_Z;
	int16_t Gyro_X;
	int16_t Gyro_Y;
	int16_t Gyro_Z;
	int8_t T;
	uint16_t TS;
	uint64_t timestamp;
} IIM42652_FIFO_PACKET_16_ST;

typedef struct
{
	uint32_t depth : 24;
	uint32_t theta_hi : 8;
	uint32_t theta_lo : 12;
	uint32_t phi : 20;
	uint8_t reflectivity;
	uint8_t tag;
} BlueSeaLidarSpherPoint;

typedef struct
{
	uint16_t mirror_rpm;
	uint16_t motor_rpm_x10;
    uint8_t tag;
    uint8_t rain;
	
} RuntimeInfoV1;
typedef struct
{
	uint8_t version;
	uint16_t length;
	uint16_t time_interval; /**< unit: 0.1 us */
	uint16_t dot_num;
	uint16_t udp_cnt;
	uint8_t frame_cnt;
	uint8_t data_type;
	uint8_t time_type;
	union
	{
		RuntimeInfoV1 rt_v1;
		uint8_t rsvd[12];
	};
	uint32_t crc32;
	uint64_t timestamp;
	uint8_t data[0]; /**< Point cloud data. */
					 // BlueSeaLidarSpherPoint points[BLUESEA_PAC_POINT];
} BlueSeaLidarEthernetPacket;

struct LidarMsgHdr
{
	char sign[4];
	uint32_t proto_version;
	char dev_sn[20];
	uint32_t dev_id;
	uint32_t timestamp;
	uint32_t flags;
	uint32_t events;
	uint16_t id;
	uint16_t extra;
	uint32_t zone_actived;
	uint8_t all_states[32];
	uint32_t reserved[11];
};
#pragma pack(pop)

typedef struct
{
	uint8_t version;
	uint32_t length;
	uint16_t time_interval; /**< unit: 0.1 us */
	uint16_t dot_num;
	uint16_t udp_cnt;
	uint8_t frame_cnt;
	uint8_t data_type;
	uint8_t time_type;
	uint8_t rsvd[12];
	uint32_t crc32;
	uint64_t timestamp;
	uint8_t data[0]; /**< Point cloud data. */
} LidarPacketData;

typedef struct
{
	uint32_t offset_time; // offset time relative to the base time
	float x;			  // X axis, unit:m
	float y;			  // Y axis, unit:m
	float z;			  // Z axis, unit:m
	uint8_t reflectivity; // reflectivity, 0~255
	uint8_t tag;		  // bluesea tag
	uint8_t line;		  // laser number in lidar
} LidarCloudPointData;

typedef struct
{
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float acc_x;
	float acc_y;
	float acc_z;
	float linear_acceleration_x;
	float linear_acceleration_y;
	float linear_acceleration_z;
} LidarImuPointData;

typedef enum
{
	LIDARIMUDATA = 0,
	LIDARPOINTCLOUD = 0x01,
} LidarPointDataType;

typedef struct
{
	uint32_t second;
	uint32_t nano_second;
} TIME_ST;

struct DevHeart
{
	char sign[4];
	uint32_t proto_version;
	uint32_t timestamp[2];
	char dev_sn[20];
	char dev_type[16];
	uint32_t version;
	uint32_t dev_id;
	uint8_t ip[4];
	uint8_t mask[4];
	uint8_t gateway[4];
	uint8_t remote_ip[4];
	uint16_t remote_port;
	uint16_t port;
	uint16_t state;
	uint16_t motor_rpm; // 0.1
	uint16_t mirror_rpm;
	uint16_t lrf_version; // 0.1
	uint16_t temperature; // 0.1
	uint16_t voltage;	  // 0.001
	char alarm[16];
	uint32_t crc;
};

struct EEpromV101
{
	char label[4];
	uint16_t pp_ver;
	uint16_t size;
	uint8_t dev_sn[20];
	uint8_t dev_type[16];
	uint32_t dev_id;
	// network
	uint8_t IPv4[4];
	uint8_t mask[4];
	uint8_t gateway[4];
	uint8_t srv_ip[4];
	uint16_t srv_port;
	uint16_t local_port;
	char reserved[9];
	uint8_t target_fixed;
	uint8_t reserved2[74];
};

struct KeepAlive
{
	uint32_t world_clock;
	uint32_t mcu_hz;
	uint32_t arrive;
	uint32_t delay;
	uint32_t reserved[4];
};


typedef struct
{
	int sfp_enable;
	int window;		  // 阴影检测窗口大小
	double min_angle; // 最小角度
	double max_angle; // 最大角度
	double effective_distance;
} ShadowsFilterParam;

typedef struct
{
	int dfp_enable;
	int continuous_times; // 持续帧数
	double dirty_factor;  // 脏污点报警系数

} DirtyFilterParam;
typedef struct
{
	int mr_enable;
	float roll;
	float pitch;
	float yaw;
	float x;
	float y;
	float z;
} MatrixRotate;

typedef struct
{
	int mr_enable;
	double trans[3];
	double rotation[3][3];
} MatrixRotate_2;

typedef struct
{
	std::string lidar_ip;
	int lidar_port;
	int listen_port;
	int ptp_enable;

	int frame_package_num;
	int timemode;
} ArgData;

typedef void (*LidarCloudPointCallback)(uint32_t handle, const uint8_t dev_type, const LidarPacketData *data, void *client_data);
typedef void (*LidarImuDataCallback)(uint32_t handle, const uint8_t dev_type, const LidarPacketData *data, void *client_data);
typedef void (*LidarLogDataCallback)(uint32_t handle, const uint8_t dev_type, const char *data, int len);
typedef void (*LidarAlarmCallback)(uint32_t handle, const uint8_t dev_type, const char *data, int len);

#endif
