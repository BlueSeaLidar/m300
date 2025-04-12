#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp> // 用于日志回调
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include "bluesea_m300_inter/msg/custom_msg.hpp"
#include "bluesea_m300_inter/msg/custom_point.hpp"
#include "bluesea_m300_inter/srv/control.hpp"
#include "../sdk/sdk.h" // 假设SDK头文件路径正确

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
struct ArgData
{
  std::string frame_id;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud;
  std::string topic_pointcloud;
  bool output_pointcloud;

  rclcpp::Publisher<bluesea_m300_inter::msg::CustomMsg>::SharedPtr pub_custommsg;
  std::string topic_custommsg;
  bool output_custommsg;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
  std::string topic_imu;
  bool output_imu;

  std::string lidar_ip;
  int lidar_port;
  int local_port;
  int ptp_enable;

  int frame_package_num;
  int timemode;
};

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LidarPacketData *data, void *client_data)
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
    ArgData *argdata = (ArgData *)client_data;
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
      bluesea_m300_inter::msg::CustomMsg msg;
      uint16_t N = data->dot_num;
      msg.point_num = N;
      msg.lidar_id = 0;
      msg.header.frame_id = argdata->frame_id;

      msg.header.stamp.sec = data->timestamp / 1000000000;
      msg.header.stamp.nanosec = data->timestamp % 1000000000;

      msg.rsvd = {0, 0, 0};
      for (size_t i = 0; i < N; i++)
      {
        bluesea_m300_inter::msg::CustomPoint point;
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
  free(data);
}
void ImuDataCallback(uint32_t handle, const uint8_t dev_type, LidarPacketData *data, void *client_data)
{
  if (data == nullptr)
  {
    return;
  }
  // printf("Imu data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
  //        handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
  if (data->data_type == LIDARIMUDATA)
  {
    ArgData *argdata = (ArgData *)client_data;
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

  free(data);
}
void LogDataCallback(uint32_t handle, const uint8_t dev_type, char *data, int len)
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
      : Node("lidar_m300"), argdata_{}
  {
    // 读取参数
    READ_PARAM(std::string, "frame_id", argdata_.frame_id, "map");
    READ_PARAM(std::string, "topic_pointcloud", argdata_.topic_pointcloud, "pointcloud");
    READ_PARAM(bool, "output_pointcloud", argdata_.output_pointcloud, true);
    READ_PARAM(std::string, "topic_custommsg", argdata_.topic_custommsg, "custommsg");
    READ_PARAM(bool, "output_custommsg", argdata_.output_custommsg, true);
    READ_PARAM(std::string, "topic_imu", argdata_.topic_imu, "imu");
    READ_PARAM(bool, "output_imu", argdata_.output_imu, true);
    READ_PARAM(std::string, "lidar_ip", argdata_.lidar_ip, "192.168.158.98");
    READ_PARAM(int, "lidar_port", argdata_.lidar_port, 6668);
    READ_PARAM(int, "local_port", argdata_.local_port, 6668);
    READ_PARAM(int, "ptp_enable", argdata_.ptp_enable, -1);
    READ_PARAM(int, "frame_package_num", argdata_.frame_package_num, 180);
    READ_PARAM(int, "timemode", argdata_.timemode, 1);

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

    // 创建发布者
    if (argdata_.output_pointcloud)
    {
      argdata_.pub_pointcloud = create_publisher<sensor_msgs::msg::PointCloud2>(
          argdata_.topic_pointcloud, 10);
    }
    if (argdata_.output_custommsg)
    {
      argdata_.pub_custommsg = create_publisher<bluesea_m300_inter::msg::CustomMsg>(
          argdata_.topic_custommsg, 10);
    }
    if (argdata_.output_imu)
    {
      argdata_.pub_imu = create_publisher<sensor_msgs::msg::Imu>(
          argdata_.topic_imu, 10);
    }

    // 初始化SDK并设置回调
    BlueSeaLidarSDK::getInstance()->Init();
    devID = BlueSeaLidarSDK::getInstance()->AddLidar(
        argdata_.lidar_ip.c_str(), argdata_.lidar_port, argdata_.local_port, argdata_.ptp_enable, argdata_.frame_package_num,argdata_.timemode, sfp, dfp);

    BlueSeaLidarSDK::getInstance()->SetPointCloudCallback(devID, PointCloudCallback, &argdata_);
    BlueSeaLidarSDK::getInstance()->SetImuDataCallback(devID, ImuDataCallback, &argdata_);
    BlueSeaLidarSDK::getInstance()->SetLogDataCallback(devID, LogDataCallback, nullptr);

    // 连接激光雷达（可能需要循环直到成功）
    // while (BlueSeaLidarSDK::getInstance()->read_calib(
    //            argdata_.lidar_ip.c_str(), argdata_.lidar_port) != 0)
    while (BlueSeaLidarSDK::getInstance()->read_calib(
               argdata_.lidar_ip.c_str(), argdata_.lidar_port) != 0)
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for lidar calibration...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    BlueSeaLidarSDK::getInstance()->ConnectLidar(devID);
    // 创建服务端
    service = this->create_service<bluesea_m300_inter::srv::Control>("scan_conntrol", std::bind(&LidarNode::control, this, _1, _2));
  }

private:
  rclcpp::Service<bluesea_m300_inter::srv::Control>::SharedPtr service;
  ArgData argdata_;
  int devID;
  void control(const bluesea_m300_inter::srv::Control::Request::SharedPtr req, const bluesea_m300_inter::srv::Control::Response::SharedPtr res)
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
    res->code = BlueSeaLidarSDK::getInstance()->SetLidarAction(devID, action);
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
