/*If you are using a third-party queue library instead of the queue wheel in the demo, just replace the contents of the macro definition CUSTOM_WHELL*/

#include "../sdk/pacecatlidarsdk.h"
#include "../sdk/global.h"
#include <sstream>

#include "../3rdparty/readerwriterqueue/readerwriterqueue.h"
using namespace moodycamel;

//多雷达宏定义
#define DUAL_LIDAR
ReaderWriterQueue<std::string> g_pointcloud_queue;
ReaderWriterQueue<std::string> g_imu_queue;

#ifdef DUAL_LIDAR
ReaderWriterQueue<std::string> g_pointcloud_queue2;
ReaderWriterQueue<std::string> g_imu_queue2;
#endif

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, const LidarPacketData *data, void *client_data)
{
	if (data == nullptr)
	{
		return;
	}

#ifdef CUSTOM_WHELL
	if (handle == 0)
	{
		std::string chunk((char *)data, sizeof(LidarPacketData) + sizeof(LidarCloudPointData) * data->dot_num);
		g_pointcloud_queue.try_enqueue(chunk);
	}
	else if (handle == 1)
	{
		std::string chunk((char *)data, sizeof(LidarPacketData) + sizeof(LidarCloudPointData) * data->dot_num);
		g_pointcloud_queue2.try_enqueue(chunk);
	}
	/*printf("workthread id: %d  point cloud handle: %u, data_num: %d, data_type: %d, length: %d, frame_counter: %d\n",
		std::this_thread::get_id(), handle, data->dot_num, data->data_type, data->length, data->frame_cnt);*/
#endif
}

void ImuDataCallback(uint32_t handle, const uint8_t dev_type, const LidarPacketData *data, void *client_data)
{
	if (data == nullptr)
	{
		return;
	}
#ifdef CUSTOM_WHELL
	if (handle == 0)
	{
		std::string chunk((char *)data, sizeof(LidarPacketData) + sizeof(LidarImuPointData));
		g_imu_queue.try_enqueue(chunk);
	}
	else if (handle == 1)
	{
		std::string chunk((char *)data, sizeof(LidarPacketData) + sizeof(LidarImuPointData));
		g_imu_queue2.try_enqueue(chunk);
	}
	/*printf("Imu data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
	handle, data->dot_num, data->data_type, data->length, data->frame_cnt);*/
#endif
}

void LogDataCallback(uint32_t handle, const uint8_t dev_type, const char *data, int len)
{
	if (data == nullptr)
	{
		return;
	}
	if (dev_type != MSG_CRITICAL)
	{
		char tmp[256] = {0};
		memcpy(tmp, data, len);
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

	std::string adapter = "ens38";
	ShadowsFilterParam sfp;
	sfp.sfp_enable = 0;
	sfp.window = 1;
	sfp.min_angle = 5.0;
	sfp.max_angle = 175.0;
	sfp.effective_distance = 5.0;

	DirtyFilterParam dfp;
	dfp.dfp_enable = 0;
	dfp.continuous_times = 30;
	dfp.dirty_factor = 0.01;

	MatrixRotate mr;
	mr.mr_enable = 0;
	mr.roll = 0.0;
	mr.pitch = 0.0;
	mr.yaw = 0.0;
	mr.x = 0.0;
	mr.y = 0.0;
	mr.z = 0.0;
	MatrixRotate_2 mr_2;
	setMatrixRotateParam(mr, mr_2);

	PaceCatLidarSDK::getInstance()->Init(adapter);
#if 1
	ArgData argdata;
	argdata.lidar_ip="192.168.158.10";
	argdata.lidar_port = 6543;
	argdata.listen_port = 6668;
	argdata.ptp_enable = 0;
	argdata.frame_package_num = 180;
	argdata.timemode = 0;

	int devID = PaceCatLidarSDK::getInstance()->AddLidar(argdata, sfp, dfp, mr_2);

	PaceCatLidarSDK::getInstance()->SetPointCloudCallback(devID, PointCloudCallback, nullptr);
	PaceCatLidarSDK::getInstance()->SetImuDataCallback(devID, ImuDataCallback, nullptr);
	PaceCatLidarSDK::getInstance()->SetLogDataCallback(devID, LogDataCallback, nullptr);
	PaceCatLidarSDK::getInstance()->SetAlarmDataCallback(devID, AlarmDataCallback, nullptr);
	bool ret = PaceCatLidarSDK::getInstance()->ConnectLidar(devID);
	if (!ret)
		return -1;
#endif

#if 0
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
	/*******************quert encoding_disk info**********************************/

	std::string encoding_disk_info;
	result = PaceCatLidarSDK::getInstance()->QueryMCUInfo(devID,encoding_disk_info);
	if (result)
		printf("QueryMCUInfo:query encoding_disk_info  ok:%s\n",encoding_disk_info.c_str());

#endif
#ifdef DUAL_LIDAR
	// multiple lidars  ,please make sure lidar ip and   localport  is must be not same
	ArgData argdata2;
	argdata2.lidar_ip="192.168.158.96";
	argdata2.lidar_port = 6546;
	argdata2.listen_port = 6669;
	argdata2.ptp_enable = 0;
	argdata2.frame_package_num = 180;
	argdata2.timemode = 0;
	int devID2 = PaceCatLidarSDK::getInstance()->AddLidar(argdata2, sfp, dfp, mr_2);
	PaceCatLidarSDK::getInstance()->SetPointCloudCallback(devID2, PointCloudCallback, nullptr);
	PaceCatLidarSDK::getInstance()->SetImuDataCallback(devID2, ImuDataCallback, nullptr);
	PaceCatLidarSDK::getInstance()->SetLogDataCallback(devID2, LogDataCallback, nullptr);
	PaceCatLidarSDK::getInstance()->ConnectLidar(devID2);
#endif
	while (1)
	{
		std::string chunk;
		bool ret = g_pointcloud_queue.try_dequeue(chunk);
		if (ret)
		{
			// LidarPacketData *data = (LidarPacketData *)(chunk.c_str());
			// std::ostringstream oss;
			// oss << std::this_thread::get_id();
			// printf("data:main thread:%s ,data_num: %d, data_type: %d, length: %d, frame_counter: %d\n",
			// 	   oss.str().c_str(), data->dot_num, data->data_type, data->length, data->frame_cnt);

			// printf every points xyz data
			/*LidarCloudPointData* p_point_data = (LidarCloudPointData*)data->data;
			for (uint32_t i = 0; i < data->dot_num; i++) {
				printf("%f %f %f\n", p_point_data[i].x, p_point_data[i].y, p_point_data[i].z);
				p_point_data[i].x;
				p_point_data[i].y;
				p_point_data[i].z;
			}*/
		}
		ret = g_imu_queue.try_dequeue(chunk);
		if (ret)
		{
			// LidarPacketData *data = (LidarPacketData *)(chunk.c_str());
			// // LidarImuPointData* imudata = (LidarImuPointData*)data->data;
			// std::ostringstream oss;
			// oss << std::this_thread::get_id();
			// printf("imu queue:main thread:%s,data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
			// oss.str().c_str(), data->dot_num, data->data_type, data->length, data->frame_cnt);
			// printf imu data
			// printf("%f %f %f %f %f %f\n", imudata->acc_x, imudata->acc_y, imudata->acc_z, imudata->gyro_x, imudata->gyro_y, imudata->gyro_z);
		}
#ifdef DUAL_LIDAR
		ret = g_pointcloud_queue2.try_dequeue(chunk);
		if (ret)
		{
		}
		ret = g_imu_queue2.try_dequeue(chunk);
		if (ret)
		{
		}
#endif
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	PaceCatLidarSDK::getInstance()->Uninit();
}
