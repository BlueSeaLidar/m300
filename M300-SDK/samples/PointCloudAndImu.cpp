/*If you are using a third-party queue library instead of the queue wheel in the demo, just replace the contents of the macro definition CUSTOM_WHELL*/

#include "../sdk/pacecatlidarsdk.h"
#include"../sdk/global.h"

#define CUSTOM_WHELL

#ifdef CUSTOM_WHELL
#include"../3rdparty/readerwriterqueue/readerwriterqueue.h"
using namespace moodycamel;
ReaderWriterQueue<std::string>* g_pointcloud_queue;
ReaderWriterQueue<std::string>* g_imu_queue;

#endif // DEBUG

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, const LidarPacketData *data, void *client_data)
{
	if (data == nullptr)
	{
		return;
	}
#ifdef CUSTOM_WHELL
	std::string chunk((char*)data,sizeof(LidarPacketData) + sizeof(LidarCloudPointData) * data->dot_num);
	//g_pointcloud_queue->try_enqueue(chunk);
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
	std::string chunk((char*)data, sizeof(LidarPacketData) + sizeof(LidarImuPointData));
	g_imu_queue->try_enqueue(chunk);
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
	printf("ID::%d print level:%d msg:%s\n", handle, dev_type, data);
}

int main()
{
#ifdef CUSTOM_WHELL
	ReaderWriterQueue<std::string> q(20);
	ReaderWriterQueue<std::string> q2(20);
	g_pointcloud_queue = &q;
	g_imu_queue = &q2;
#endif
	ArgData argdata;
	char lidar_addr[] = "192.168.0.210";
	memcpy(argdata.lidar_ip,lidar_addr,strlen(lidar_addr)+1);
	argdata.lidar_port = 6543;
	argdata.listen_port = 6668;
	argdata.ptp_enable = -1;
	argdata.frame_package_num = 180;
	argdata.timemode=1;

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
	mr.mr_enable=0;
	mr.roll=0.0;
	mr.pitch=0.0;
	mr.yaw=0.0;
	mr.x=0.0;
	mr.y=0.0;
	mr.z=0.0;
	MatrixRotate_2 mr_2;
	setMatrixRotateParam(mr, mr_2);

	PaceCatLidarSDK::getInstance()->Init();
	int devID = PaceCatLidarSDK::getInstance()->AddLidar(argdata,sfp, dfp,mr_2);

	PaceCatLidarSDK::getInstance()->SetPointCloudCallback(devID, PointCloudCallback, nullptr);
	PaceCatLidarSDK::getInstance()->SetImuDataCallback(devID, ImuDataCallback, nullptr);
	PaceCatLidarSDK::getInstance()->SetLogDataCallback(devID, LogDataCallback, nullptr);
	while (PaceCatLidarSDK::getInstance()->read_calib(argdata.lidar_ip, argdata.lidar_port) != 0);
	PaceCatLidarSDK::getInstance()->ConnectLidar(devID);

	// multiple lidars  ,please make sure lidar ip and   localport  is must be not same

	// char lidar_addr2[] = "192.168.1.10";
	// int lidar_port2 = 6543;
	// int listen_port2 = 6669;
	// int devID2 = PaceCatLidarSDK::getInstance()->AddLidar(lidar_addr2, lidar_port2, listen_port2);
	// PaceCatLidarSDK::getInstance()->SetPointCloudCallback(devID2, PointCloudCallback, nullptr);
	// PaceCatLidarSDK::getInstance()->SetImuDataCallback(devID2, ImuDataCallback, nullptr);
	// PaceCatLidarSDK::getInstance()->SetLogDataCallback(devID2, LogDataCallback, nullptr);
	// PaceCatLidarSDK::getInstance()->ConnectLidar(devID2);

	while (1)
	{
		//std::this_thread::sleep_for(std::chrono::milliseconds(1));
#ifdef CUSTOM_WHELL
		std::string chunk;
		bool ret = g_pointcloud_queue->try_dequeue(chunk);
		if (ret)
		{
			LidarPacketData* data = (LidarPacketData*)(chunk.c_str());
			printf("data:main thread:%u ,data_num: %d, data_type: %d, length: %d, frame_counter: %d\n",
				std::this_thread::get_id(), data->dot_num, data->data_type, data->length, data->frame_cnt);

			//printf every points xyz data
			/*LidarCloudPointData* p_point_data = (LidarCloudPointData*)data->data;
			for (uint32_t i = 0; i < data->dot_num; i++) {
				printf("%f %f %f\n", p_point_data[i].x, p_point_data[i].y, p_point_data[i].z);
				p_point_data[i].x;
				p_point_data[i].y;
				p_point_data[i].z;
			}*/
		}
		ret = g_imu_queue->try_dequeue(chunk);
		if (ret)
		{
			LidarPacketData* data = (LidarPacketData*)(chunk.c_str());
			LidarImuPointData* imudata = (LidarImuPointData*)data->data;
			printf("imu queue:main thread:%u,data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
				std::this_thread::get_id(), data->dot_num, data->data_type, data->length, data->frame_cnt);
			//printf imu data
			//printf("%f %f %f %f %f %f\n", imudata->acc_x, imudata->acc_y, imudata->acc_z, imudata->gyro_x, imudata->gyro_y, imudata->gyro_z);
		}
#endif
		
	}
	PaceCatLidarSDK::getInstance()->Uninit();
}
