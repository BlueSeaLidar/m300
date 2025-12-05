// M300_SDK.h: 标准系统包含文件的包含文件
// 或项目特定的包含文件。
#pragma once

#include<vector>
#include<queue>
#include<thread>
#include<mutex>
#include<string>
#ifdef _WIN32
#pragma warning(disable : 4996)
#pragma warning(disable : 4244)
#include <winsock2.h>
#include <iphlpapi.h>
#pragma comment(lib, "iphlpapi.lib")
#pragma comment(lib, "ws2_32.lib")
typedef uint32_t in_addr_t;
#else
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>
#endif
#include "../3rdparty/readerwriterqueue/readerwriterqueue.h"
#include"protocol.h"
#include"global.h"
#include"event.h"

#define M300_E_SDKVERSION "V1.6.5_2025120401" // SDK版本号

#define DEBUG_TEST 1
#define LOG_TIMER 2
#define RECV_OK "OK"
#define RECV_NG "NG"

typedef struct
{
	std::string uuid;
	std::string model;
	std::string lidarip;
	std::string lidarmask;
	std::string lidargateway;
	uint16_t lidarport;
	std::string uploadip;
	uint16_t uploadport;
	uint8_t uploadfix;

}BaseInfo;

typedef struct
{
	std::string mcu_ver;
	std::string motor_ver;
	std::string software_ver;
}VersionInfo;

typedef struct
{
	std::string sn;
	std::string ip;
	int port;
	uint64_t timestamp;
	int flag;//during a time and not recv heart package
	uint16_t motor_rpm; // 0.1
	uint16_t mirror_rpm;//1
	uint16_t temperature; // 0.1
	uint16_t voltage;	  // 0.001
	bool isonline;
}ConnectInfo;
typedef struct
{
	std::vector<ConnectInfo>lidars;
	int code;
	std::string value;
	bool isrun;
	std::string adapter;
}HeartInfo;

struct CmdTask
{
    uint64_t send_timestamp;//发送的时间戳
    uint8_t tried;//已经尝试次数
    std::string cmd;//测试指令
	int cmd_type;//指令类型    C_PACK,S_PACK,GS_PACK
	uint16_t rand;//随机码
};
struct CmdTaskList
{
	uint8_t max_waittime;//最大等待时间 单位:秒
	uint8_t max_try_count;//最大重试次数
	std::queue<CmdTask>cmdtask;//任务列表
};

enum LidarState
{
	OFFLINE = 0,
	ONLINE,
	QUIT
};
enum LidarAction
{
	NONE,
	WAIT,
	FINISH,
	START,
	STOP,
	RESTART,
	GET_PARAMS,
	GET_VERSION,
	SET_NETWORK,
	SET_UPLOAD_NETWORK,
	SET_UPLOAD_FIX,
	PTP_INIT,
	GET_NETERR,
	UPGRADE,
	CACHE_CLEAR,
	GET_DIRTY,
	GET_ENCODING_DISK,
	GET_ERRLIST,
	CLR_ERRLIST,
	GET_ADCINFO,
	CMD_TALK
};
enum LidarMsg
{
	MSG_DEBUG,
	MSG_WARM,
	MSG_ERROR,
	MSG_CRITICAL

};
enum CriticalMSG
{
	ERR_TEMPERATURE_HIGH=0,
	ERR_MOTOR_ZERO,
	ERR_MIRROR_ZERO,
	ERR_VOLTAGE_LOW,
	ERR_VOLTAGE_HIGH,
	ERR_MOTOR_NO_STABLE,
	ERR_MIRROR_NO_STABLE,
	ERR_DATA_ZERO
};

//运行配置 
struct RunConfig
{
	int ID;
	std::thread  thread_data;
	std::thread  thread_cmd;
	//std::thread  thread_pubCloud;
	//std::thread  thread_pubImu;
	LidarCloudPointCallback  cb_cloudpoint= nullptr;
	void *cloudpoint;
	LidarImuDataCallback cb_imudata = nullptr;
	void *imudata;
	LidarLogDataCallback cb_logdata = nullptr;;
	void*logdata;
	LidarAlarmCallback cb_alarmdata = nullptr;
	void*alarmdata;
	LidarState run_state;
	std::string lidar_ip;
	int lidar_port;
	int listen_port;
	std::vector<LidarCloudPointData> cloud_data;
	std::queue<IIM42652_FIFO_PACKET_16_ST> imu_data;
	IMUDrift  imu_drift;
	uint32_t frame_cnt;
	uint64_t frame_firstpoint_timestamp;  //everyframe  first point timestamp
	LidarAction action;
	int send_len;
	char send_buf[256]={0};
	int send_type;
	int recv_len;
	char recv_buf[2048]={0};
	int ptp_enable;
	bool cache_clear{false}; 
	ShadowsFilterParam sfp ;
	DirtyFilterParam   dfp;
	MatrixRotate_2 mr_2;
	int frame_package_num;
	uint16_t package_num_idx{0};//当前帧计数
	int timemode;
	uint8_t rain;
	uint8_t echo_mode;
	std::string log_path;
};
struct UserHeartInfo
{
	float motor_rpm; // 0.1
	uint16_t mirror_rpm;
	float temperature; // 0.1
	float voltage;	  // 0.001
	uint8_t isonline;//  offline/online
	uint64_t timestamp;
	int code;
	std::string value;

};

struct DeBugInfo
{
	uint64_t pointcloud_timestamp_large_last;//最后一次时间间隔过大的时间戳
	uint32_t pointcloud_timestamp_large_num;//当前时间段打印次数统计

	uint64_t pointcloud_timestamp_jumpback_last;//最后一次时间回跳的时间戳
	uint32_t pointcloud_timestamp_jumpback_num;//当前时间段打印次数统计
	
	uint64_t pointcloud_timestamp_drop_last;//最后一次丢包的时间戳
	uint32_t pointcloud_timestamp_drop_num;//当前时间段打印次数统计

	uint64_t pointcloud_timestamp_last; // 雷达最后一次更新时间戳
	int16_t  pointcloud_packet_idx;//点云包下标

	uint64_t imu_timestamp_large_last;//最后一次时间间隔过大的时间戳
	uint32_t imu_timestamp_large_num;//当前时间段打印次数统计

	uint64_t imu_timestamp_jumpback_last;//最后一次时间回跳的时间戳
	uint32_t imu_timestamp_jumpback_num;//当前时间段打印次数统计

	uint64_t imu_timestamp_drop_last;//最后一次丢imu包的时间戳
	uint32_t imu_timestamp_drop_num;//当前时间段打印次数统计

	uint64_t imu_timestamp_last; // 雷达最后一次更新时间戳
	int16_t  imu_packet_idx;//imu包下标

	uint64_t  timer;//定时检测时间暂定1S

	uint64_t pointcloud_imu_timestamp_large_last;//点云时间戳间隔过大报警
	uint32_t pointcloud_imu_timestamp_large_num;//当前时间段打印次数统计

	uint64_t mirror_err_timestamp_last;//转镜异常时间戳
	uint32_t mirror_err_timestamp_num;//当前时间段打印次数统计

	uint64_t motor_err_timestamp_last;//底板异常时间戳
	uint32_t motor_err_timestamp_num;//当前时间段打印次数统计


	uint64_t dirtydata_err_timestamp_last;//脏污数据异常时间戳
	uint64_t dirtydata_err_timestamp_num;

	uint64_t filterdata_err_timestamp_last;//去拖点数量异常时间戳
	uint64_t filterdata_err_timestamp_num;

	uint64_t zero_pointdata_timestamp_last;//0点过多报警时间戳
	uint64_t zero_pointdata_timestamp_num;

	uint64_t distance_close_timestamp_last;//距离过近系数报警时间戳
	uint64_t distance_close_timestamp_num;
};


class PaceCatLidarSDK
{
public:
	static PaceCatLidarSDK *getInstance();
	static void deleteInstance();

	/*
	*    Bind the network card for communication
	*/
	void Init(std::string adapter);
	void Uninit();

	/*
	 *	callback function  get pointcloud data  imu data  logdata
	 */
	bool SetPointCloudCallback(int ID, LidarCloudPointCallback cb, void* client_data);
	bool SetImuDataCallback(int ID, LidarImuDataCallback cb, void* client_data);
	bool SetLogDataCallback(int ID, LidarLogDataCallback cb, void* client_data);
	bool SetAlarmDataCallback(int ID, LidarAlarmCallback cb, void* client_data);

	void WritePointCloud(int ID, const uint8_t dev_type, LidarPacketData *data);
	void WriteImuData( int ID, const uint8_t dev_type, LidarPacketData* data);
	void WriteLogData(int ID, const uint8_t dev_type, char* data, int len);
	void WriteAlarmData(int ID, const uint8_t dev_type, char* data, int len);

	/*
	 *	add lidar by lidar ip    lidar port    local listen port
	 */
	int AddLidar(ArgData argdata,ShadowsFilterParam sfp,DirtyFilterParam dfp,MatrixRotate_2 mr_2);
	/*
	*   add lidar by  playback
	*/
	int AddLidarForPlayback(std::string logpath,int frame_rate);
	/*
	*   add lidar by  upgrade
	*/
	int AddLidarForUpgrade(std::string lidar_ip,int lidar_port,int listen_port);
	/*
	 *	connect lidar     send cmd/parse recvice data
	 */
	bool ConnectLidar(int ID,bool isplayback=false);
	/*
	 *	disconnect lidar,cancel listen lidar
	 */
	bool DisconnectLidar(int ID);

	/*
	 *	query connect lidar base info
	 */
	bool QueryBaseInfo(int ID, BaseInfo &info);

	/*
	 *	query connect lidar version
	 */
	bool QueryVersion(int ID, VersionInfo &info);

	/*
	 *	use by lidar heart  query lidar  state
	 */
	UserHeartInfo QueryDeviceState(int ID);

	/*
	 *	set lidar    ip  mask  gateway  receive port
	 */
	bool SetLidarNetWork(int ID,std::string ip, std::string mask, std::string gateway, uint16_t port);


	/*
	 *	set lidar    upload ip    upload port     fixed upload or not
	 */
	bool SetLidarUploadNetWork(int ID, std::string upload_ip, uint16_t upload_port);

	/*
	 *	set lidar    start work     stop work（sleep）  restart work(reboot)
	 */
	bool SetLidarAction(int ID, int action);
	/*
	 *	set lidar  firmware upgrade
	 */
	bool SetLidarUpgrade(int ID, std::string path);

	/*
	 *	set lidar  ptp init
	 */
	bool SetLidarPTPInit(int ID);
	/*
	 *	query lidar  network err  netinfo
	 */
	bool QueryLidarNetWork(int ID,std::string& netinfo);
	/*
	 *	clear frame cache (Applied to situations   powered on or off, or rpm is unstable)
	 */
	void ClearFrameCache(int ID);

	/*
	 *	query lidar  dirty data total data,please use it at begin and error time
	 */
	bool QueryDirtyData(int ID,std::string &dirty_data);
	/*
	 *	query lidar  encoding disk info
	 */
	bool QueryMCUInfo(int ID,std::string &encoding_disk_info);

	/*
	 *	query lidar  errinfo list
	 */
	bool QueryLidarErrList(int ID,std::string& errlist);
	/*
	 *	clear lidar  errinfo list
	 */
	bool CleanLidarErrList(int ID);

	/*
	 *	get rain data
	 */
	bool QueryRainData(int ID,uint8_t&rain);
	/*
	 *	get echo mode
	 */
	bool QueryEchoMode(int ID,uint8_t&echo_mode);
		/*
	*   query high temperature  adc err code 
 	*/
 	bool QueryADCInfo(int ID,std::string& adcinfo);

	bool ReadCalib(int ID,std::string lidar_ip, int port);

	int QueryIDByIp(std::string ip);
	
private:
	void UDPDataThreadProc(int id);
	void ParseLogThreadProc(int id);
	void PlaybackThreadProc(int id);
	void HeartThreadProc(HeartInfo &heartinfo);

	void UDPCmdThreadProc(int id);

	int PackNetCmd(uint16_t type, uint16_t len, uint16_t sn, const void* buf, uint8_t* netbuf);
	int SendNetPack(int sock, uint16_t type, uint16_t len, const void* buf, char*ip, int port);
	void AddPacketToList(const BlueSeaLidarEthernetPacket* bluesea, std::vector<LidarCloudPointData>& cloud_data, uint64_t first_timestamp, double& last_ang, std::vector<LidarCloudPointData>& tmp_filter, std::vector<double>& tmp_ang, ShadowsFilterParam& sfp, MatrixRotate_2 mr_2);
	double PacketToPoints(BlueSeaLidarSpherPoint bluesea, LidarCloudPointData& point);

	RunConfig* GetConfig(int ID);
	bool FirmwareUpgrade(int ID,std::string  ip, int port,int listenport,std::string path);

private:
	static PaceCatLidarSDK *m_sdk;
	PaceCatLidarSDK();
	~PaceCatLidarSDK();

	int m_idx;
	std::vector<RunConfig*> m_lidars;
	std::thread m_heartthread;
	HeartInfo m_heartinfo;
#if DEBUG_TEST
	uint32_t m_zero_point_num{0};
	uint32_t m_distance_close_num{0};
	uint32_t m_sum_point_num{0};
	uint32_t m_filter_num{0};
#endif
//playback
	moodycamel::ReaderWriterQueue<std::string> m_log_queue;
};
double getAngleWithViewpoint(float r1, float r2, double included_angle);
int ShadowsFilter(std::vector<LidarCloudPointData> &scan_in,std::vector<double> &ang_in,const ShadowsFilterParam& param,std::vector<double> &tmp_ang);
bool isBitSet(uint8_t num, int n);
uint16_t Decode(uint16_t n, const uint8_t* buf, std::queue<IIM42652_FIFO_PACKET_16_ST>& imu_data/*, std::mutex &imu_mutex*/);
in_addr_t get_interface_ip(const char* ifname);
std::string bin_to_hex_fast(const uint8_t* data, size_t length, bool uppercase = true);








