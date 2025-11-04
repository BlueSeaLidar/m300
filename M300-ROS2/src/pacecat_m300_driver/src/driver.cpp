#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include "pacecat_m300_inter/msg/custom_msg.hpp"
#include "pacecat_m300_inter/msg/custom_point.hpp"
#include "pacecat_m300_inter/srv/control.hpp"
#include "../sdk/pacecatlidarsdk.h"
#include "../sdk/global.h"
enum PointField
{
  INT8 = 1,
  UINT8,
  INT16,
  UINT16,
  INT32,
  UINT32,
  FLOAT32,
  FLOAT64
};

using namespace std::placeholders;
typedef struct
{
  std::string frame_id;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud;
  std::string topic_pointcloud;
  bool output_pointcloud;

  rclcpp::Publisher<pacecat_m300_inter::msg::CustomMsg>::SharedPtr pub_custommsg;
  std::string topic_custommsg;
  bool output_custommsg;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
  std::string topic_imu;
  bool output_imu;
  std::string adapter;
} PubTopic;

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, const LidarPacketData *data, void *client_data)
{
  if (data == nullptr)
  {
    return;
  }
  // printf("point cloud handle: %u, data_num: %d, data_type: %d, length: %d, frame_counter: %d\n",
  // handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
  if (data->data_type == LIDARPOINTCLOUD)
  {
    LidarCloudPointData *p_point_data = (LidarCloudPointData *)data->data;
    PubTopic *argdata = (PubTopic *)client_data;
    if (argdata->output_pointcloud)
    {
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header.frame_id.assign(argdata->frame_id);
      cloud.height = 1;
      cloud.width = data->dot_num;
      cloud.fields.resize(7);
      cloud.fields[0].offset = 0;
      cloud.fields[0].name = "x";
      cloud.fields[0].count = 1;
      cloud.fields[0].datatype = PointField::FLOAT32;

      cloud.fields[1].offset = 4;
      cloud.fields[1].name = "y";
      cloud.fields[1].count = 1;
      cloud.fields[1].datatype = PointField::FLOAT32;

      cloud.fields[2].offset = 8;
      cloud.fields[2].name = "z";
      cloud.fields[2].count = 1;
      cloud.fields[2].datatype = PointField::FLOAT32;

      cloud.fields[3].offset = 12;
      cloud.fields[3].name = "intensity";
      cloud.fields[3].count = 1;
      cloud.fields[3].datatype = PointField::FLOAT32;

      cloud.fields[4].offset = 16;
      cloud.fields[4].name = "tag";
      cloud.fields[4].count = 1;
      cloud.fields[4].datatype = PointField::UINT8;

      cloud.fields[5].offset = 17;
      cloud.fields[5].name = "line";
      cloud.fields[5].count = 1;
      cloud.fields[5].datatype = PointField::UINT8;

      cloud.fields[6].offset = 18;
      cloud.fields[6].name = "timestamp";
      cloud.fields[6].count = 1;
      cloud.fields[6].datatype = PointField::FLOAT64;
      cloud.point_step = 26;
      cloud.row_step = cloud.width * cloud.point_step;
      cloud.data.resize(cloud.row_step * cloud.height);

      for (size_t i = 0; i < data->dot_num; i++)
      {
        memcpy(&cloud.data[0] + i * 26, &p_point_data[i].x, 4);
        memcpy(&cloud.data[0] + i * 26 + 4, &p_point_data[i].y, 4);
        memcpy(&cloud.data[0] + i * 26 + 8, &p_point_data[i].z, 4);
        float reflectivity = p_point_data[i].reflectivity;
        memcpy(&cloud.data[0] + i * 26 + 12, &reflectivity, 4);
        memcpy(&cloud.data[0] + i * 26 + 16, &p_point_data[i].tag, 1);
        memcpy(&cloud.data[0] + i * 26 + 17, &p_point_data[i].line, 1);

        double offset_time = p_point_data[i].offset_time;
        memcpy(&cloud.data[0] + i * 26 + 18, &offset_time, 8);
      }
      cloud.header.stamp.sec = data->timestamp / 1000000000;
      cloud.header.stamp.nanosec = data->timestamp % 1000000000;

      argdata->pub_pointcloud->publish(cloud);
    }
    if (argdata->output_custommsg)
    {
      pacecat_m300_inter::msg::CustomMsg msg;
      uint16_t N = data->dot_num;
      msg.point_num = N;
      msg.lidar_id = 0;
      msg.header.frame_id = argdata->frame_id;
      msg.timebase = data->timestamp;
      msg.header.stamp.sec = data->timestamp / 1000000000;
      msg.header.stamp.nanosec = data->timestamp % 1000000000;

      msg.rsvd = {0, 0, 0};
      for (size_t i = 0; i < N; i++)
      {
        pacecat_m300_inter::msg::CustomPoint point;
        point.x = p_point_data[i].x;
        point.y = p_point_data[i].y;
        point.z = p_point_data[i].z;
        point.reflectivity = p_point_data[i].reflectivity;
        point.offset_time = p_point_data[i].offset_time;
        point.line = p_point_data[i].line;
        point.tag = p_point_data[i].tag;
        msg.points.push_back(point);
      }
      argdata->pub_custommsg->publish(msg);
    }
  }
}
void ImuDataCallback(uint32_t handle, const uint8_t dev_type, const LidarPacketData *data, void *client_data)
{
  if (data == nullptr)
  {
    return;
  }
  // printf("Imu data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
  //        handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
  if (data->data_type == LIDARIMUDATA)
  {
    PubTopic *argdata = (PubTopic *)client_data;
    if (argdata->output_imu)
    {
      LidarImuPointData *p_imu_data = (LidarImuPointData *)data->data;
      sensor_msgs::msg::Imu imu;
      imu.angular_velocity.x = p_imu_data->gyro_x;
      imu.angular_velocity.y = p_imu_data->gyro_y;
      imu.angular_velocity.z = p_imu_data->gyro_z;

      imu.linear_acceleration.x = p_imu_data->linear_acceleration_x;
      imu.linear_acceleration.y = p_imu_data->linear_acceleration_y;
      imu.linear_acceleration.z = p_imu_data->linear_acceleration_z;

      uint64_t nanosec = data->timestamp;
      imu.header.frame_id = argdata->frame_id;

      imu.header.stamp.sec = nanosec / 1000000000;
      imu.header.stamp.nanosec = nanosec % 1000000000;
      argdata->pub_imu->publish(imu);
    }
  }
}
void LogDataCallback(uint32_t handle, const uint8_t dev_type, const char *data, int len)
{
  if (data == nullptr)
  {
    return;
  }
  printf("ID::%d print level:%d msg:%s\n", handle, dev_type, data);
}

#define READ_PARAM(TYPE, NAME, VAR, INIT) \
  VAR = INIT;                             \
  declare_parameter<TYPE>(NAME, VAR);     \
  get_parameter(NAME, VAR);

class LidarNode : public rclcpp::Node
{
public:
  LidarNode()
      : Node("lidar_m300")
  {
    // 读取参数
    // PubTopic pubtopic;
    READ_PARAM(std::string, "frame_id", pubtopic.frame_id, "map");
    READ_PARAM(std::string, "topic_pointcloud", pubtopic.topic_pointcloud, "pointcloud");
    READ_PARAM(bool, "output_pointcloud", pubtopic.output_pointcloud, true);
    READ_PARAM(std::string, "topic_custommsg", pubtopic.topic_custommsg, "custommsg");
    READ_PARAM(bool, "output_custommsg", pubtopic.output_custommsg, true);
    READ_PARAM(std::string, "topic_imu", pubtopic.topic_imu, "imu");
    READ_PARAM(bool, "output_imu", pubtopic.output_imu, true);
    READ_PARAM(std::string, "adapter", pubtopic.adapter, "eth0");

    ArgData argdata;
    READ_PARAM(std::string, "lidar_ip", argdata.lidar_ip, "192.168.158.98");
    READ_PARAM(int, "lidar_port", argdata.lidar_port, 6543);
    READ_PARAM(int, "local_port", argdata.listen_port, 6668);
    READ_PARAM(int, "ptp_enable", argdata.ptp_enable, -1);
    READ_PARAM(int, "frame_package_num", argdata.frame_package_num, 180);
    READ_PARAM(int, "timemode", argdata.timemode, 1);

    ShadowsFilterParam sfp;
    READ_PARAM(int, "sfp_enable", sfp.sfp_enable, 1);
    READ_PARAM(int, "window", sfp.window, 1);
    READ_PARAM(double, "min_angle", sfp.min_angle, 5.0);
    READ_PARAM(double, "max_angle", sfp.max_angle, 175.0);
    READ_PARAM(double, "effective_distance", sfp.effective_distance, 5.0);

    DirtyFilterParam dfp;
    READ_PARAM(int, "dfp_enable", dfp.dfp_enable, 1);
    READ_PARAM(int, "continuous_times", dfp.continuous_times, 30);
    READ_PARAM(double, "dirty_factor", dfp.dirty_factor, 0.005);

    MatrixRotate mr;
    READ_PARAM(int, "mr_enable", mr.mr_enable, 0);
    READ_PARAM(float, "roll", mr.roll, static_cast<float>(0.0));
    READ_PARAM(float, "pitch", mr.pitch, static_cast<float>(0.0));
    READ_PARAM(float, "yaw", mr.yaw, static_cast<float>(0.0));
    READ_PARAM(float, "x", mr.x, static_cast<float>(0.0));
    READ_PARAM(float, "y", mr.y, static_cast<float>(0.0));
    READ_PARAM(float, "z", mr.z, static_cast<float>(0.0));
    MatrixRotate_2 mr_2;
    setMatrixRotateParam(mr, mr_2);

    // 创建发布者
    if (pubtopic.output_pointcloud)
    {
      pubtopic.pub_pointcloud = create_publisher<sensor_msgs::msg::PointCloud2>(
          pubtopic.topic_pointcloud, 10);
    }
    if (pubtopic.output_custommsg)
    {
      pubtopic.pub_custommsg = create_publisher<pacecat_m300_inter::msg::CustomMsg>(
          pubtopic.topic_custommsg, 10);
    }
    if (pubtopic.output_imu)
    {
      pubtopic.pub_imu = create_publisher<sensor_msgs::msg::Imu>(
          pubtopic.topic_imu, 10);
    }

    // 初始化SDK并设置回调
    PaceCatLidarSDK::getInstance()->Init(pubtopic.adapter);
    devID = PaceCatLidarSDK::getInstance()->AddLidar(argdata, sfp, dfp, mr_2);

    PaceCatLidarSDK::getInstance()->SetPointCloudCallback(devID, PointCloudCallback, &pubtopic);
    PaceCatLidarSDK::getInstance()->SetImuDataCallback(devID, ImuDataCallback, &pubtopic);
    PaceCatLidarSDK::getInstance()->SetLogDataCallback(devID, LogDataCallback, nullptr);
    PaceCatLidarSDK::getInstance()->ConnectLidar(devID);
    // 创建服务端
    service = this->create_service<pacecat_m300_inter::srv::Control>("scan_conntrol", std::bind(&LidarNode::control, this, _1, _2));
  }

private:
  rclcpp::Service<pacecat_m300_inter::srv::Control>::SharedPtr service;
  // ArgData argdata;
  PubTopic pubtopic;
  int devID;
  void control(const pacecat_m300_inter::srv::Control::Request::SharedPtr req, const pacecat_m300_inter::srv::Control::Response::SharedPtr res)
  {
    LidarAction action = LidarAction::NONE;

    if (req->func == "start")
    {
      action = LidarAction::START;
    }
    else if (req->func == "stop")
    {
      action = LidarAction::STOP;
    }
    RCLCPP_INFO(this->get_logger(), "set lidar action: %s", req->func.c_str());
    res->code = PaceCatLidarSDK::getInstance()->SetLidarAction(devID, action);
    if (res->code <= 0)
      res->value = "faild";
    else if (res->code == 1)
      res->value = "OK";
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarNode>());
  rclcpp::shutdown();
  return 0;
}
