// M300_SDK.h: 标准系统包含文件的包含文件
// 或项目特定的包含文件。
#pragma once

#include<vector>
#include<queue>
#include<thread>
#include<mutex>
#include<string>
#include"protocol.h"

#define M300_E_SDKVERSION "V1.4_20250403" // SDK版本号


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
	bool state;
	int flag;
}ConnectInfo;
typedef struct
{
	std::vector<ConnectInfo>lidars;
	int code;
	std::string value;
	bool isrun;
}HeartInfo;

enum LidarState
{
	OFFLINE = 0,
	ONLINE,
	QUIT
};
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
enum LidarMsg
{
	MSG_DEBUG,
	MSG_WARM,
	MSG_ERROR

};

//运行配置 
struct RunConfig
{
	int ID;
	std::thread  thread_subData;
	//std::thread  thread_pubCloud;
	//std::thread  thread_pubImu;
	LidarCloudPointCallback  cb_cloudpoint;
	void *cloudpoint;
	LidarImuDataCallback cb_imudata;
	void *imudata;
	LidarLogDataCallback cb_logdata;
	void*logdata;
	int run_state;
	std::string lidar_ip;
	int lidar_port;
	int listen_port;
	std::vector<LidarCloudPointData> cloud_data;
	//std::vector<LidarCloudPointData> cloud_data_cache;

	std::queue<IIM42652_FIFO_PACKET_16_ST> imu_data;
	//std::mutex data_mutex;
	//std::mutex imu_mutex;
	IMUDrift  imu_drift;
	uint32_t frame_cnt;
	uint64_t frame_firstpoint_timestamp;  //everyframe  first point timestamp
	//uint64_t frame_firstpoint_timestamp_cache;
	int action;
	int send_len;
	std::string send_buf;
	int recv_len;
	std::string recv_buf;
	int ptp_enable;
	ShadowsFilterParam sfp ;
	DirtyFilterParam   dfp;
	int frame_package_num;
};
class BlueSeaLidarSDK
{
public:
	static BlueSeaLidarSDK *getInstance();
	static void deleteInstance();

	void Init();
	void Uninit();

	/*
	 *	callback function  get pointcloud data  imu data  logdata
	 */
	bool SetPointCloudCallback(int ID, LidarCloudPointCallback cb, void* client_data);
	bool SetImuDataCallback(int ID, LidarImuDataCallback cb, void* client_data);
	bool SetLogDataCallback(int ID, LidarLogDataCallback cb, void* client_data);

	void WritePointCloud(int ID, const uint8_t dev_type, LidarPacketData *data);
	void WriteImuData( int ID, const uint8_t dev_type, LidarPacketData* data);
	void WriteLogData(int ID, const uint8_t dev_type, char* data, int len);


	/*
	 *	add lidar by lidar ip    lidar port    local listen port
	 */
	int AddLidar(std::string lidar_ip, int lidar_port, int listen_port,int ptp_enable,int frame_package_num,ShadowsFilterParam sfp,DirtyFilterParam dfp);
	/*
	 *	connect lidar     send cmd/parse recvice data
	 */
	bool ConnectLidar(int ID);
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
	int QueryDeviceState(int ID);

	/*
	 *	set lidar    ip  mask  gateway  receive port
	 */
	bool SetLidarNetWork(int ID,std::string ip, std::string mask, std::string gateway, uint16_t port);


	/*
	 *	set lidar    upload ip    upload port     fixed upload or not
	 */
	bool SetLidarUploadNetWork(int ID, std::string upload_ip, uint16_t upload_port);

	/*
	 *	set lidar    upload ip    upload port     fixed upload or not
	 */
	bool SetLidarUploadFix(int ID, bool isfix);


	/*
	 *	set lidar    start work     stop work（sleep）  restart work(reboot)
	 */
	bool SetLidarAction(int ID, int action);
	/*
	 *	set lidar  firmware upgrade
	 */
	bool SetLidarUpgrade(int ID, std::string path);



	int read_calib(const char* lidar_ip, int port);
	int QueryIDByIp(std::string ip);
	RunConfig*getConfig(int ID);
	uint16_t Decode(uint16_t n, const uint8_t* buf, std::queue<IIM42652_FIFO_PACKET_16_ST>&imu_data/*, std::mutex &imu_mutex*/);
	void AddPacketToList(const BlueSeaLidarEthernetPacket* bluesea, std::vector<LidarCloudPointData> &cloud_data,uint64_t first_timestamp,double &last_ang,std::vector<LidarCloudPointData> &tmp_filter,std::vector<double> &tmp_ang,ShadowsFilterParam &sfp);
	bool firmwareUpgrade_udp(std::string  ip, int port, std::string path, std::string &error);
protected:

private:
	double PacketToPoints(BlueSeaLidarSpherPoint bluesea, LidarCloudPointData &point);

	int PackNetCmd(uint16_t type, uint16_t len, uint16_t sn, const void* buf, uint8_t* netbuf);
	int SendNetPack(int sock, uint16_t type, uint16_t len, const void* buf, char*ip, int port);
	
	
	
private:
	static BlueSeaLidarSDK *m_sdk;
	BlueSeaLidarSDK();
	~BlueSeaLidarSDK();

	int m_idx;
	std::vector<RunConfig*> m_lidars;
	uint64_t m_npoint = 0;
	uint64_t m_npub = 0;
	bool bottom_ccw = false;
	uint64_t last_ns = 0;
	int64_t sidx = 0;
	int64_t imu_idx = 0;
	int m_currentframeidx{ 0 };
	std::thread m_heartthread;
	HeartInfo m_heartinfo;
};

void UDPThreadProc(int id);
void PubCloudThreadProc(int id);
void PubImuThreadProc(int id);
void HeartThreadProc(HeartInfo &heartinfo);


FirmwareFile* LoadFirmware(const char* path);
void SendUpgradePack(unsigned int udp, const FirmwarePart* fp, char* ip, int port, int SN, ResendPack *resndBuf);
double getAngleWithViewpoint(float r1, float r2, double included_angle);
int ShadowsFilter(std::vector<LidarCloudPointData> &scan_in,std::vector<double> &ang_in,const ShadowsFilterParam& param,std::vector<double> &tmp_ang);
bool isBitSet(uint8_t num, int n);








