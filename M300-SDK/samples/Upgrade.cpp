#include "../sdk/pacecatlidarsdk.h"
#include"../sdk/global.h"

//集成使用以下数据需在以下回调函数中加锁同步
void LogDataCallback(uint32_t handle, const uint8_t dev_type, const char* data, int len) {
	if (data == nullptr) {
		return;
	}
	if (dev_type != MSG_CRITICAL)
	{
		char tmp[256]={0};
		memcpy(tmp,data,len);
		printf("ID::%d print level:%d msg:%s\n", handle, dev_type, tmp);
	}
}

int main()
{
	std::string lidar_ip = "192.168.0.199";
	int lidar_port = 6543;
	int listen_port = 6777;//不与数据监听端口相同即可
    //std::string  upgrade_file_path = "/home/pacecat/wangzn/ota/3d_m300_mcu_motor/test/LIDAR_M300-1206-20250822-192403_dual.lhr";
	//std::string  upgrade_file_path = "/home/pacecat/wangzn/ota/3d_m300_mcu_motor/test/LIDAR_M300-20250916-104712-dual.lhr";
	std::string  upgrade_file_path = "/home/pacecat/wangzn/ota/3d_m300_mcu_motor/test/LDS-M300-E-20250829-161051.lhl";
	std::string  upgrade_file_path2 = "/home/pacecat/wangzn/ota/3d_m300_mcu_motor/test/LDS-M300-E-20250919-150234.lhl";
	int devID = PaceCatLidarSDK::getInstance()->AddLidarForUpgrade(lidar_ip,lidar_port,listen_port);
	PaceCatLidarSDK::getInstance()->SetLogDataCallback(devID, LogDataCallback, nullptr);
	bool ret = PaceCatLidarSDK::getInstance()->SetLidarUpgrade(devID, upgrade_file_path);
	printf("lidar upgrade %d\n", ret);
	sleep(20);
	bool ret2 = PaceCatLidarSDK::getInstance()->SetLidarUpgrade(devID, upgrade_file_path2);
	printf("lidar upgrade2 %d\n", ret2);

	PaceCatLidarSDK::getInstance()->Uninit();
	return 0;
}