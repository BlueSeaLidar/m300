#include "../sdk/pacecatlidarsdk.h"
#include "../sdk/global.h"
void LogDataCallback(uint32_t handle, const uint8_t dev_type, const char *data, int len)
{
	if (data == nullptr)
	{
		return;
	}
	if (dev_type != MSG_CRITICAL)
	{
		char tmp[256]={0};
		memcpy(tmp,data,len);
		printf("ID::%d print level:%d msg:%s\n", handle, dev_type, tmp);
	}
}
void AlarmDataCallback(uint32_t handle, const uint8_t dev_type, const char *data, int len)
{
	if (data == nullptr)
	{
		return;
	}
	uint32_t flag = 0;
	memcpy(&flag, data, len);
	// 判定条件:温度高于85  底板转速和机头转速为0  电压过低或者过高:[10,32]
	if (getbit(flag, ERR_TEMPERATURE_HIGH))
	{
		printf("ERR_TEMPERATURE_HIGH\n");
	}
	if (getbit(flag, ERR_MOTOR_ZERO))
	{
		printf("ERR_MOTOR_ZERO\n");
	}
	if (getbit(flag, ERR_MIRROR_ZERO))
	{
		printf("ERR_MIRROR_ZERO\n");
	}
	if (getbit(flag, ERR_VOLTAGE_LOW))
	{
		printf("ERR_VOLTAGE_LOW\n");
	}
	if (getbit(flag, ERR_VOLTAGE_HIGH))
	{
		printf("ERR_VOLTAGE_HIGH\n");
	}
	if (getbit(flag, ERR_MOTOR_NO_STABLE))
	{
		printf("ERR_MOTOR_NO_STABLE\n");
	}
	if (getbit(flag, ERR_MIRROR_NO_STABLE))
	{
		printf("TAG_MIRROR_NOT_STABLE\n");
	}
	if (getbit(flag, ERR_DATA_ZERO))
	{
		printf("too many zero point num, scale factor over 0.8\n");
	}
}
int main()
{
	ArgData argdata;
	argdata.lidar_ip="192.168.0.199";
	argdata.lidar_port = 6543;
	argdata.listen_port = 6668;
	argdata.ptp_enable = 0;
	argdata.frame_package_num = 180;
	argdata.timemode = 0;
	std::string adapter = "ens38";

	ShadowsFilterParam sfp;
	sfp.sfp_enable = 0;
	sfp.window = 1;
	sfp.min_angle = 5.0;
	sfp.max_angle = 175.0;
	sfp.effective_distance = 5.0;
	DirtyFilterParam dfp;
	dfp.dfp_enable = 0;
	MatrixRotate mr;
	mr.mr_enable = 0;
	MatrixRotate_2 mr_2;
	setMatrixRotateParam(mr, mr_2);

	PaceCatLidarSDK::getInstance()->Init(adapter);
	int devID = PaceCatLidarSDK::getInstance()->AddLidar(argdata, sfp, dfp, mr_2);

	PaceCatLidarSDK::getInstance()->SetLogDataCallback(devID, LogDataCallback, nullptr);
	PaceCatLidarSDK::getInstance()->SetAlarmDataCallback(devID, AlarmDataCallback, nullptr);

	bool ret = PaceCatLidarSDK::getInstance()->ConnectLidar(devID);
	if(!ret)
		return -1;

#if 1
	bool result = false;
	/*****************query lidar base info**************************/
	BaseInfo info;
	result = PaceCatLidarSDK::getInstance()->QueryBaseInfo(devID, info);
	if (result)
	{
		printf(" ID:%d uuid:%s  model:%s\n lidarip:%s lidarmask:%s lidargateway:%s lidarport:%d \n uploadip:%s uploadport:%d  uploadfix:%d\n",
			   devID,
			   info.uuid.c_str(), info.model.c_str(),
			   info.lidarip.c_str(), info.lidarmask.c_str(), info.lidargateway.c_str(), info.lidarport,
			   info.uploadip.c_str(), info.uploadport, info.uploadfix);
	}
	else
	{
		printf("QueryBaseInfo:query base info err\n");
	}

	/*****************query lidar version**************************/
	VersionInfo  info2;
	PaceCatLidarSDK::getInstance()->QueryVersion(devID, info2);
	printf("QueryVersion:ID:%d mcu_ver:%s  motor_ver:%s\n software_ver:%s \n", devID, info2.mcu_ver.c_str(), info2.motor_ver.c_str(), info2.software_ver.c_str());

	/*****************set lidar ptp restart**************************/
	result = PaceCatLidarSDK::getInstance()->SetLidarPTPInit(devID);
	if (result)
		printf("SetLidarPTPInit:ptp init ok\n");

	/*******************quert dirty data**********************************/
	std::string dirty_data;
	result = PaceCatLidarSDK::getInstance()->QueryDirtyData(devID,dirty_data);
	if (result)
		printf("QueryDirtyData:query dirty data ok:%s\n",dirty_data.c_str());


	/*******************quert encoding_disk info**********************************/
	std::string encoding_disk_info;
	result = PaceCatLidarSDK::getInstance()->QueryMCUInfo(devID,encoding_disk_info);
	if (result)
		printf("QueryMCUInfo:query encoding_disk_info  ok:%s\n",encoding_disk_info.c_str());
	
	/*******************quert network info**********************************/
	std::string netinfo;
	bool isok = PaceCatLidarSDK::getInstance()->QueryLidarNetWork(devID, netinfo);
	printf("QueryLidarNetWork:ID:%d  result:%d  %s\n",devID, isok, netinfo.c_str());


	/*******************quert and clean errlist**********************************/
	std::string errlist;
	isok = PaceCatLidarSDK::getInstance()->QueryLidarErrList(devID, errlist);
	printf("QueryLidarErrList:ID:%d  result:%d \n",devID, isok);

	isok = PaceCatLidarSDK::getInstance()->CleanLidarErrList(devID);
	printf("CleanLidarErrList:ID:%d  result:%d \n",devID, isok);

	/*******************query adc info**********************************/

	std::string adcinfo;
	bool adcinfo_isok = PaceCatLidarSDK::getInstance()->QueryADCInfo(devID,adcinfo);
	printf("QueryADCInfo:ID:%d  adcinfo result:%d  %s \n",devID, adcinfo_isok,adcinfo.c_str());

	//int timeout=100;
#endif
#if 0
	/*****************set action  work   or not work **************************/
	bool isok = PaceCatLidarSDK::getInstance()->SetLidarAction(devID, STOP);
	printf("STOP:ID:%d  result:%d \n", devID, isok);
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
	bool isok2 = PaceCatLidarSDK::getInstance()->SetLidarAction(devID, START);
	printf("START:ID:%d  result:%d \n", devID, isok2);
#endif

#if 0
	/*****************set lidar network**************************/
	char lidar_addr2[] = "192.168.0.199";
	char mask2[] = "255.255.255.0";
	char gateway2[] = "192.168.0.1";
	int lidar_port2 = 6543;
	PaceCatLidarSDK::getInstance()->SetLidarNetWork(devID,lidar_addr2, mask2, gateway2, lidar_port2);
#endif
#if 0
/*****************set lidar upload  network**************************/
	char lidar_addr3[] = "192.168.0.123";
	int lidar_port3 = 6668;
	bool ret3=PaceCatLidarSDK::getInstance()->SetLidarUploadNetWork(devID, lidar_addr3,lidar_port3);
	printf("SetLidarUploadNetWork:ID:%d  set upload network:%d \n", devID, ret3);
#endif

	while (1)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		/*****************query  lidar is online**************************/
		UserHeartInfo heartinfo = PaceCatLidarSDK::getInstance()->QueryDeviceState(devID);
		printf("QueryDeviceState:ID:%d code:%d value:%s isOnline:%d  temperature:%.1f  motor_rpm:%.1f  mirror_rpm:%d voltage:%.3f timestamp:%ld\n",
			   devID, heartinfo.code, heartinfo.value.c_str(), heartinfo.isonline, heartinfo.temperature, heartinfo.motor_rpm, heartinfo.mirror_rpm, heartinfo.voltage, heartinfo.timestamp);
		if (!heartinfo.isonline)
		{
			PaceCatLidarSDK::getInstance()->ClearFrameCache(devID);
		}
		uint8_t rain;
		PaceCatLidarSDK::getInstance()->QueryRainData(devID,rain);
		printf("ID:%d  rain:%d \n", devID,(int)rain);

		uint8_t echo_mode;
		PaceCatLidarSDK::getInstance()->QueryEchoMode(devID,echo_mode);
		printf("ID:%d  echo_mode:%d \n", devID,(int)echo_mode);

	}
	return 0;
	
}
