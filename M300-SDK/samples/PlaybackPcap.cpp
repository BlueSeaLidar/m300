#include <iostream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <math.h>
#include <thread>
#include <mutex>
#include <queue>
#include "../sdk/pacecatlidarsdk.h"
#include "../sdk/global.h"
#define CUSTOM_WHELL

#ifdef CUSTOM_WHELL
#include "../3rdparty/readerwriterqueue/readerwriterqueue.h"
using namespace moodycamel;
ReaderWriterQueue<std::string> g_pointcloud_queue;
ReaderWriterQueue<std::string> g_imu_queue;

#endif // DEBUG

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, const LidarPacketData *data, void *client_data)
{
  if (data == nullptr)
  {
    return;
  }
#ifdef CUSTOM_WHELL
  std::string chunk((char *)data, sizeof(LidarPacketData) + sizeof(LidarCloudPointData) * data->dot_num);
  g_pointcloud_queue.try_enqueue(chunk);
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
  std::string chunk((char *)data, sizeof(LidarPacketData) + sizeof(LidarImuPointData));
  g_imu_queue.try_enqueue(chunk);
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
    printf("ID::%d print level:%d msg:%s\n", handle, dev_type, data);
  else
  {
    uint32_t flag = 0;
    memcpy(&flag, data, len);
    // 判定条件:温度高于85  底板转速和机头转速为0  电压过低或者过高:[10,32]
    if (getbit(data[0], ERR_TEMPERATURE_HIGH))
    {
      printf("ERR_TEMPERATURE_HIGH\n");
    }
    if (getbit(data[0], ERR_MOTOR_ZERO))
    {
      printf("ERR_MOTOR_ZERO\n");
    }
    if (getbit(data[0], ERR_MIRROR_ZERO))
    {
      printf("ERR_MIRROR_ZERO\n");
    }
    if (getbit(data[0], ERR_VOLTAGE_LOW))
    {
      printf("ERR_VOLTAGE_LOW\n");
    }
    if (getbit(data[0], ERR_VOLTAGE_HIGH))
    {
      printf("ERR_VOLTAGE_HIGH\n");
    }
  }
}

int main(int argc, char **argv)
{

  std::string log_path = "/home/pacecat/wangzn/logdata/test.pcap";
  int frame_packet_num = 150; // 一帧的包数
  int devID = PaceCatLidarSDK::getInstance()->AddLidarForPlayback(log_path, frame_packet_num);
  PaceCatLidarSDK::getInstance()->SetPointCloudCallback(devID, PointCloudCallback, nullptr);
  PaceCatLidarSDK::getInstance()->SetImuDataCallback(devID, ImuDataCallback, nullptr);
  PaceCatLidarSDK::getInstance()->SetLogDataCallback(devID, LogDataCallback, nullptr);
  PaceCatLidarSDK::getInstance()->ConnectLidar(devID, true);

  while (1)
  {
#ifdef CUSTOM_WHELL
    std::string chunk;
    bool ret = g_pointcloud_queue.try_dequeue(chunk);
    if (ret)
    {
      LidarPacketData *data = (LidarPacketData *)(chunk.c_str());
      std::ostringstream oss;
      oss << std::this_thread::get_id();
      printf("data:main thread:%s ,data_num: %d, data_type: %d, length: %d, frame_counter: %d\n",
      oss.str().c_str(), data->dot_num, data->data_type, data->length, data->frame_cnt);

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
      //LidarPacketData *data = (LidarPacketData *)(chunk.c_str());
      // LidarImuPointData* imudata = (LidarImuPointData*)data->data;
      //std::ostringstream oss;
      //oss << std::this_thread::get_id();
      // printf("imu queue:main thread:%s,data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
      // oss.str().c_str(), data->dot_num, data->data_type, data->length, data->frame_cnt);
      // printf imu data
      // printf("%f %f %f %f %f %f\n", imudata->acc_x, imudata->acc_y, imudata->acc_z, imudata->gyro_x, imudata->gyro_y, imudata->gyro_z);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
#endif
  }
  PaceCatLidarSDK::getInstance()->Uninit();

  return 0;
}
