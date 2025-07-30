#include "../sdk/pacecatlidarsdk.h"
#include"../sdk/global.h"
void LogDataCallback(uint32_t handle, const uint8_t dev_type, const char* data, int len) {
	if (data == nullptr) {
		return;
	}
	printf("ID::%d print level:%d msg:%s\n", handle, dev_type, data);
}

int main()
{
	ArgData argdata;
	char lidar_addr[] = "192.168.0.88";
	memcpy(argdata.lidar_ip,lidar_addr,strlen(lidar_addr)+1);
	argdata.lidar_port = 6543;
	argdata.listen_port = 6668;
	argdata.ptp_enable = 0;
	argdata.frame_package_num = 180;
	argdata.timemode=0;


	ShadowsFilterParam sfp;
	sfp.sfp_enable=0;
	sfp.window=1;
	sfp.min_angle=5.0;
	sfp.max_angle=175.0;
	sfp.effective_distance=5.0;
	DirtyFilterParam dfp;
	dfp.dfp_enable=0;
	MatrixRotate mr;
	mr.mr_enable=0;
	MatrixRotate_2 mr_2;
	setMatrixRotateParam(mr, mr_2);

	PaceCatLidarSDK::getInstance()->Init();
	int devID = PaceCatLidarSDK::getInstance()->AddLidar(argdata,sfp,dfp,mr_2);

	PaceCatLidarSDK::getInstance()->SetLogDataCallback(devID, LogDataCallback, nullptr);
	PaceCatLidarSDK::getInstance()->ConnectLidar(devID);
	/*****************query lidar base info**************************/
	BaseInfo info;	
	PaceCatLidarSDK::getInstance()->QueryBaseInfo(devID, info);
	printf(" ID:%d uuid:%s  model:%s\n lidarip:%s lidarmask:%s lidargateway:%s lidarport:%d \n uploadip:%s uploadport:%d  uploadfix:%d\n",
		devID,
		info.uuid.c_str(), info.model.c_str(),
		info.lidarip.c_str(), info.lidarmask.c_str(),info.lidargateway.c_str(), info.lidarport,
		info.uploadip.c_str(),info.uploadport, info.uploadfix);


	/*****************query lidar version**************************/
	// VersionInfo  info2;
	// PaceCatLidarSDK::getInstance()->QueryVersion(devID, info2);
	// printf("ID:%d mcu_ver:%s  motor_ver:%s\n software_ver:%s \n", devID, info2.mcu_ver.c_str(), info2.motor_ver.c_str(), info2.software_ver.c_str());
	

	/*****************set action  work   or not work **************************/
	// bool isok = PaceCatLidarSDK::getInstance()->SetLidarAction(devID, STOP);
	// std::this_thread::sleep_for(std::chrono::milliseconds(3000));
	// bool isok2 = PaceCatLidarSDK::getInstance()->SetLidarAction(devID, START);
	// printf("ID:%d  action:%d  %d\n", devID, isok, isok2);


	/*****************set lidar network**************************/
	/*char lidar_addr2[] = "192.168.0.232";
	char mask2[] = "255.255.255.0";
	char gateway2[] = "192.168.0.1";
	int lidar_port2 = 6543;
	int listen_port2 = 6668;
	PaceCatLidarSDK::getInstance()->SetLidarNetWork(devID,lidar_addr2, mask2, gateway2, lidar_port2);*/

	/*****************set lidar upload  network**************************/
	/*char lidar_addr3[] = "192.168.0.47";
	int lidar_port3 = 6668;
	bool ret3=PaceCatLidarSDK::getInstance()->SetLidarUploadNetWork(devID, lidar_addr3,lidar_port3);
	bool ret4 = PaceCatLidarSDK::getInstance()->SetLidarUploadFix(devID,true);
	printf("ID:%d  set upload network:%d  %d\n", devID, ret3, ret4);*/
	bool ret = PaceCatLidarSDK::getInstance()->SetLidarPTPInit(devID);
	if(ret)
		printf("ptp init ok\n");

	std::string netinfo;
	bool isok = PaceCatLidarSDK::getInstance()->QueryLidarNetWork(devID, netinfo);
	printf("ID:%d  result:%d  %s\n",devID, isok, netinfo.c_str());


	while (1)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		/*****************query  lidar is online**************************/
		UserHeartInfo heartinfo = PaceCatLidarSDK::getInstance()->QueryDeviceState(devID);
		printf("ID:%d  isOnline:%d  temperature:%.1f  motor_rpm:%.1f  mirror_rpm:%d voltage:%.3f\n", 
		devID, heartinfo.isonline,heartinfo.temperature,heartinfo.motor_rpm,heartinfo.mirror_rpm,heartinfo.voltage);

		// bool isok = PaceCatLidarSDK::getInstance()->SetLidarAction(devID, RESTART);
		// std::this_thread::sleep_for(std::chrono::milliseconds(3000));
		// printf("ID:%d  action:%d \n", devID, isok);
		
	}
	PaceCatLidarSDK::getInstance()->Uninit();
}
