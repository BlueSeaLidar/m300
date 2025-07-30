#include "../sdk/pacecatlidarsdk.h"
#include"../sdk/global.h"

//集成使用以下数据需在以下回调函数中加锁同步
void LogDataCallback(uint32_t handle, const uint8_t dev_type, const char* data, int len) {
	if (data == nullptr) {
		return;
	}
	printf("ID::%d print level:%d msg:%s\n", handle, dev_type, data);
}

int main()
{
	ArgData argdata;
	char lidar_addr[] = "192.168.0.210";
	memcpy(argdata.lidar_ip,lidar_addr,strlen(lidar_addr)+1);
	argdata.lidar_port = 6543;
	argdata.listen_port = 6668;
	argdata.ptp_enable = 0;
	argdata.frame_package_num = 180;
	argdata.timemode=0;
	

	int ptp_enable = 1;
	int frame_package_num = 180;
	ShadowsFilterParam sfp;
	sfp.sfp_enable=0;
	DirtyFilterParam dfp;
	dfp.dfp_enable=0;
	MatrixRotate mr;
	mr.mr_enable=0;
	MatrixRotate_2 mr_2;
	setMatrixRotateParam(mr, mr_2);

	//std::string  upgrade_file_path = "C:\\Users\\49535\\Desktop\\LDS-M300-E-20241029-115237.lhl";
    std::string  upgrade_file_path = "/home/pacecat/wangzn/m300/M300-SDK/LDS-M300-E-20250121-105930.lhl";
	PaceCatLidarSDK::getInstance()->Init();
	int devID = PaceCatLidarSDK::getInstance()->AddLidar(argdata,sfp,dfp,mr_2);
	PaceCatLidarSDK::getInstance()->SetLogDataCallback(devID, LogDataCallback, nullptr);

	PaceCatLidarSDK::getInstance()->ConnectLidar(devID);


	bool ret = PaceCatLidarSDK::getInstance()->SetLidarUpgrade(devID, upgrade_file_path);
	printf(" lidar upgrade %d\n", ret);
	while (1)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	PaceCatLidarSDK::getInstance()->Uninit();
}