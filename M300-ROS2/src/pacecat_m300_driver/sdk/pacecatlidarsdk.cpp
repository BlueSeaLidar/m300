﻿// M300_SDK.cpp: 定义应用程序的入口点。
//
#include <fstream>
#include <chrono>
#include "global.h"
#include "pacecatlidarsdk.h"
#ifdef _WIN32
#pragma warning(disable : 4996)
#pragma warning(disable : 4244)
#endif

#include <iostream>
#include <chrono>
#include <ctime>
#include <sstream>
std::string GetCurrentTimeStamp(int time_stamp_type = 0)
{
	std::chrono::system_clock::time_point now = std::chrono::system_clock::now();

	std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
	std::tm *now_tm = std::localtime(&now_time_t);
	char buffer[128];
	strftime(buffer, sizeof(buffer), "%F %T", now_tm);

	std::ostringstream ss;
	ss.fill('0');

	std::chrono::milliseconds ms;
	std::chrono::microseconds cs;
	std::chrono::nanoseconds ns;
	switch (time_stamp_type)
	{
	case 0:
		ss << buffer;
		break;
	case 1:
		ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
		ss << buffer << ":" << ms.count();
		break;
	case 2:
		ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
		cs = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;
		ss << buffer << ":" << ms.count() << ":" << cs.count() % 1000;
		break;
	case 3:
		ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
		cs = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;
		ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()) % 1000000000;
		ss << buffer << ":" << ms.count() << ":" << cs.count() % 1000 << ":" << ns.count() % 1000;
		break;
	default:
		ss << buffer;
		break;
	}
	return ss.str();
}

PaceCatLidarSDK *PaceCatLidarSDK::m_sdk = new (std::nothrow) PaceCatLidarSDK();
PaceCatLidarSDK *PaceCatLidarSDK::getInstance()
{
	return m_sdk;
}

void PaceCatLidarSDK::deleteInstance()
{
	if (m_sdk)
	{
		delete m_sdk;
		m_sdk = NULL;
	}
}
PaceCatLidarSDK::PaceCatLidarSDK()
{
}

PaceCatLidarSDK::~PaceCatLidarSDK()
{
}
bool PaceCatLidarSDK::SetPointCloudCallback(int ID, LidarCloudPointCallback cb, void *client_data)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return false;

	lidar->cb_cloudpoint = cb;
	lidar->cloudpoint = client_data;
	return true;
}
bool PaceCatLidarSDK::SetImuDataCallback(int ID, LidarImuDataCallback cb, void *client_data)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return false;

	lidar->cb_imudata = cb;
	lidar->imudata = (char *)client_data;
	return true;
}
bool PaceCatLidarSDK::SetLogDataCallback(int ID, LidarLogDataCallback cb, void *client_data)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return false;

	lidar->cb_logdata = cb;
	lidar->logdata = (char *)client_data;
	return true;
}

void PaceCatLidarSDK::WritePointCloud(int ID, const uint8_t dev_type, LidarPacketData *data)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return;
	if (lidar->cb_cloudpoint != nullptr)
		lidar->cb_cloudpoint(ID, dev_type, data, lidar->cloudpoint);
	// else
	// 	free(data);
}

void PaceCatLidarSDK::WriteImuData(int ID, const uint8_t dev_type, LidarPacketData *data)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return;
	if (lidar->cb_imudata != nullptr)
		lidar->cb_imudata(ID, dev_type, data, lidar->imudata);
	// else
	// 	free(data);
}

void PaceCatLidarSDK::WriteLogData(int ID, const uint8_t dev_type, char *data, int len)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return;
	if (lidar->cb_logdata != nullptr)
		lidar->cb_logdata(ID, dev_type, data, len);
}

void PaceCatLidarSDK::Init()
{
	// Conflict with ros's single-process framework, temporarily disabled
	m_heartinfo.code = 0;
	m_heartinfo.isrun = true;
	m_heartthread = std::thread(&PaceCatLidarSDK::HeartThreadProc, PaceCatLidarSDK::getInstance(), std::ref(m_heartinfo));
	m_heartthread.detach();
}
void PaceCatLidarSDK::Uninit()
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		m_lidars.at(i)->run_state = QUIT;
	}
}
int PaceCatLidarSDK::AddLidar(ArgData argdata, ShadowsFilterParam sfp, DirtyFilterParam dfp, MatrixRotate_2 mr_2)
{
	RunConfig *cfg = new RunConfig;
	cfg->lidar_ip = argdata.lidar_ip;
	cfg->lidar_port = argdata.lidar_port;
	cfg->listen_port = argdata.listen_port;
	cfg->ID = m_lidars.size();
	cfg->run_state = ONLINE;
	cfg->frame_cnt = 0;
	cfg->cb_cloudpoint = NULL;
	cfg->cb_imudata = NULL;
	cfg->cb_logdata = NULL;
	cfg->frame_firstpoint_timestamp = 0;
	cfg->ptp_enable = argdata.ptp_enable;
	cfg->sfp = sfp;
	cfg->dfp = dfp;
	cfg->frame_package_num = argdata.frame_package_num;
	cfg->timemode = argdata.timemode;
	cfg->mr_2 = mr_2;
	m_lidars.push_back(cfg);
	return cfg->ID;
}
bool PaceCatLidarSDK::ConnectLidar(int ID)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return false;

	lidar->thread_subData = std::thread(&PaceCatLidarSDK::UDPThreadProc, PaceCatLidarSDK::getInstance(), ID);
	lidar->thread_subData.detach();

	// lidar->thread_pubCloud = std::thread(PubCloudThreadProc, ID);
	// lidar->thread_pubCloud.detach();

	// lidar->thread_pubImu = std::thread(PubImuThreadProc, ID);
	// lidar->thread_pubImu.detach();
	return true;
}
bool PaceCatLidarSDK::DisconnectLidar(int ID)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			m_lidars.at(i)->run_state = OFFLINE;
			return true;
		}
	}
	return false;
}

bool PaceCatLidarSDK::QueryBaseInfo(int ID, BaseInfo &info)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return false;

	lidar->send_len = 6;
	lidar->send_buf = "xxxxxx";
	lidar->action = GET_PARAMS;
	int index = CMD_REPEAT;
	while (lidar->action != FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == FINISH)
	{
		lidar->action = NONE;
		if (lidar->recv_len == sizeof(EEpromV101))
		{
			EEpromV101 *eepromv101 = (EEpromV101 *)lidar->recv_buf.c_str();
			info.uuid = BaseAPI::stringfilter((char *)eepromv101->dev_sn, 20);
			info.model = BaseAPI::stringfilter((char *)eepromv101->dev_type, 16);

			char tmp_IPv4[16] = {0};
			char tmp_mask[16] = {0};
			char tmp_gateway[16] = {0};
			char tmp_srv_ip[16] = {0};
			sprintf(tmp_IPv4, "%d.%d.%d.%d", eepromv101->IPv4[0], eepromv101->IPv4[1], eepromv101->IPv4[2], eepromv101->IPv4[3]);
			sprintf(tmp_mask, "%d.%d.%d.%d", eepromv101->mask[0], eepromv101->mask[1], eepromv101->mask[2], eepromv101->mask[3]);
			sprintf(tmp_gateway, "%d.%d.%d.%d", eepromv101->gateway[0], eepromv101->gateway[1], eepromv101->gateway[2], eepromv101->gateway[3]);
			sprintf(tmp_srv_ip, "%d.%d.%d.%d", eepromv101->srv_ip[0], eepromv101->srv_ip[1], eepromv101->srv_ip[2], eepromv101->srv_ip[3]);

			info.lidarip = tmp_IPv4;
			info.lidarmask = tmp_mask;
			info.lidargateway = tmp_gateway;
			info.lidarport = eepromv101->local_port;
			info.uploadip = tmp_srv_ip;
			info.uploadport = eepromv101->srv_port;
			info.uploadfix = eepromv101->target_fixed;
			return true;
		}
	}
	return false;
}
bool PaceCatLidarSDK::QueryVersion(int ID, VersionInfo &info)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return false;

	lidar->send_buf = "LXVERH";
	lidar->send_len = 6;
	lidar->action = GET_VERSION;
	int index = CMD_REPEAT;
	while (lidar->action != FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == FINISH)
	{
		lidar->action = NONE;
		if (lidar->recv_buf == "NG")
		{
			return false;
		}
		std::string tmp2 = lidar->recv_buf;
		size_t idx = tmp2.find(0x0d);
		size_t idx2 = tmp2.find(0x3a);
		if (idx == std::string::npos || idx == std::string::npos)
			return false;
		info.mcu_ver = tmp2.substr(idx2 + 1, idx - idx2 - 1);
		tmp2 = tmp2.substr(idx + 2);
		idx = tmp2.find(0x0d);
		idx2 = tmp2.find(0x3a);
		if (idx == std::string::npos || idx == std::string::npos)
			return false;
		info.motor_ver = tmp2.substr(idx2 + 1, idx - idx2 - 1);
		info.software_ver = M300_E_SDKVERSION;
		return true;
	}
	return false;
}
UserHeartInfo PaceCatLidarSDK::QueryDeviceState(int ID)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return UserHeartInfo{0, 0, 0, 0, 0};

	std::string ip = lidar->lidar_ip;
	for (unsigned int i = 0; i < m_heartinfo.lidars.size(); i++)
	{
		std::string tmpip = m_heartinfo.lidars[i].ip;
		if (ip == tmpip)
		{
			UserHeartInfo heartinfo;
			heartinfo.mirror_rpm = m_heartinfo.lidars[i].mirror_rpm;
			heartinfo.motor_rpm = m_heartinfo.lidars[i].motor_rpm / 10.0f;
			heartinfo.temperature = m_heartinfo.lidars[i].temperature / 10.0f;
			heartinfo.voltage = m_heartinfo.lidars[i].voltage / 1000.0f;
			heartinfo.isonline = m_heartinfo.lidars[i].isonline;
			return heartinfo;
		}
	}
	return UserHeartInfo{0, 0, 0, 0, 0};
}

bool PaceCatLidarSDK::SetLidarNetWork(int ID, std::string ip, std::string mask, std::string gateway, uint16_t port)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return false;

	char result[64] = {0};
	// 对传入的格式校验
	if (!BaseAPI::checkAndMerge(1, (char *)ip.c_str(), (char *)mask.c_str(), (char *)gateway.c_str(), port, result))
	{
		return false;
	}
	char tmp[128] = {0};
	sprintf(tmp, "LSUDP:%sH", result);
	lidar->send_len = strlen(tmp);
	lidar->send_buf = tmp;
	lidar->action = SET_NETWORK;

	int index = CMD_REPEAT;
	while (lidar->action != FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == FINISH)
	{
		lidar->lidar_ip = ip;
		lidar->lidar_port = port;
	}
	return true;
}

bool PaceCatLidarSDK::SetLidarUploadNetWork(int ID, std::string upload_ip, uint16_t upload_port)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return false;

	char result[50] = {0};
	// 对传入的格式校验
	if (!BaseAPI::checkAndMerge(0, (char *)upload_ip.c_str(), (char *)"", (char *)"", upload_port, result))
	{
		return false;
	}
	char tmp[64] = {0};
	sprintf(tmp, "LSDST:%sH", result);
	lidar->send_len = strlen(tmp);
	lidar->send_buf = tmp;
	lidar->action = SET_UPLOAD_NETWORK;

	int index = CMD_REPEAT;
	while (lidar->action != FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == FINISH)
	{
		lidar->action = NONE;
		if (lidar->recv_buf == "OK")
			return true;
	}
	return false;
}

bool PaceCatLidarSDK::SetLidarUploadFix(int ID, bool isfix)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return false;

	char tmp[64] = {0};
	sprintf(tmp, "LSTFX:%dH", isfix);
	lidar->send_len = strlen(tmp);
	lidar->send_buf = tmp;
	lidar->action = SET_UPLOAD_FIX;
	int index = CMD_REPEAT;
	while (lidar->action != FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == FINISH)
	{
		lidar->action = NONE;
		if (lidar->recv_buf == "OK")
			return true;
	}

	return false;
}

bool PaceCatLidarSDK::SetLidarAction(int ID, int action)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return false;

	lidar->send_len = 6;
	if (action == START)
		lidar->send_buf = "LSTARH";
	else if (action == STOP)
		lidar->send_buf = "LSTOPH";
	else if (action == RESTART)
		lidar->send_buf = "LRESTH";
	else
		return false;

	lidar->action = action;
	int index = CMD_REPEAT;
	while (lidar->action != FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == FINISH)
	{
		lidar->action = NONE;
		if (lidar->recv_buf == "OK")
			return true;
	}

	return false;
}

bool PaceCatLidarSDK::SetLidarUpgrade(int ID, std::string path)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return false;

	// check path is exist

	std::ifstream f(path.c_str());
	if (!f.good())
		return false;

	lidar->send_len = path.size();
	lidar->send_buf = path;
	lidar->action = UPGRADE;

	int index = CMD_REPEAT;
	while (lidar->action != FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(300));
		index--;
	}
	if (lidar->action == FINISH)
	{
		lidar->action = NONE;
		if (lidar->recv_buf == "OK")
			return true;
	}

	return false;
}

bool PaceCatLidarSDK::SetLidarPTPInit(int ID)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return false;

	char tmp[64] = {0};
	sprintf(tmp, "LPTPINITH");
	lidar->send_len = strlen(tmp);
	lidar->send_buf = tmp;
	lidar->action = PTP_INIT;
	int index = CMD_REPEAT;
	while (lidar->action != FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == FINISH)
	{
		lidar->action = NONE;
		// printf("%s %d\n",lidar->recv_buf.c_str(),lidar->recv_len);
		if (lidar->recv_buf.find("PTP sync") != std::string::npos)
			return true;
	}

	return false;
}
bool PaceCatLidarSDK::QueryLidarNetWork(int ID,std::string& netinfo)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return false;

	char tmp[64] = {0};
	sprintf(tmp, "LNETRRH");
	lidar->send_len = strlen(tmp);
	lidar->send_buf = tmp;
	lidar->action = GET_NETERR;
	int index = CMD_REPEAT;
	while (lidar->action != FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == FINISH)
	{
		lidar->action = NONE;
		if (lidar->recv_buf.find("Boot:") != std::string::npos)
		{
			//printf("%s %d\n",lidar->recv_buf.c_str(),lidar->recv_len);
			netinfo = std::string(lidar->recv_buf,0,lidar->recv_len);
			return true;
		}
	}

	return false;
}
bool PaceCatLidarSDK::ClearFrameCache(int ID)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return false;

	lidar->action = CACHE_CLEAR;
	int index = CMD_REPEAT;
	while (lidar->action != FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == FINISH)
	{
		lidar->action = NONE;
		return true;
	}

	return false;
}

int PaceCatLidarSDK::QueryIDByIp(std::string ip)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->lidar_ip == ip && m_lidars.at(i)->run_state != QUIT)
		{
			return m_lidars.at(i)->ID;
		}
	}
	return -1;
}

RunConfig *PaceCatLidarSDK::GetConfig(int ID)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			return m_lidars[i];
		}
	}
	return nullptr;
}
double PaceCatLidarSDK::PacketToPoints(BlueSeaLidarSpherPoint bluesea, LidarCloudPointData &point)
{
	point.reflectivity = bluesea.reflectivity;
	int32_t theta = bluesea.theta_hi;
	theta = (theta << 12) | bluesea.theta_lo;

	double ang = (90000 - theta) * M_PI / 180000;
	double vertical_ang = ang;
	double depth = bluesea.depth / 1000.0;

	double r = depth * cos(ang);
	point.z = depth * sin(ang);

	ang = bluesea.phi * M_PI / 180000;
	point.x = cos(ang) * r;
	point.y = sin(ang) * r;

	point.tag = 0;
	point.line = 0;
	if ((r != 0.0 && r < 0.035))
	{
		point.tag += 128; // 光学罩内的点进行标记，方便后续进行脏污判断
	}

#if DEBUG_TEST
	if (bluesea.depth == 0)
		m_zero_point_num++;
	if (bluesea.depth < 0.2 * 1000 && bluesea.depth > 0)
		m_distance_close_num++;
	m_sum_point_num++;
#endif

	return vertical_ang;
}

// int tmp2=0;
void PaceCatLidarSDK::AddPacketToList(const BlueSeaLidarEthernetPacket *packet, std::vector<LidarCloudPointData> &cloud_data, uint64_t first_timestamp, double &last_ang, std::vector<LidarCloudPointData> &tmp_filter, std::vector<double> &tmp_ang, ShadowsFilterParam &sfp, MatrixRotate_2 mr_2)
{
	std::vector<LidarCloudPointData> tmp;
	BlueSeaLidarSpherPoint *data = (BlueSeaLidarSpherPoint *)packet->data;
	for (int i = 0; i < packet->dot_num; i++)
	{
		LidarCloudPointData point;
		double ang = PacketToPoints(data[i], point);
		// todo  将点转置
		if (mr_2.mr_enable)
		{
			point.x = point.x * mr_2.rotation[0][0] + point.y * mr_2.rotation[0][1] + point.z * mr_2.rotation[0][2] + mr_2.trans[0];
			point.y = point.x * mr_2.rotation[1][0] + point.y * mr_2.rotation[1][1] + point.z * mr_2.rotation[1][2] + mr_2.trans[1];
			point.z = point.x * mr_2.rotation[2][0] + point.y * mr_2.rotation[2][1] + point.z * mr_2.rotation[2][2] + mr_2.trans[2];
		}
		point.offset_time = packet->timestamp + i * packet->time_interval * 100.0 / (packet->dot_num - 1) - first_timestamp;
		if (sfp.sfp_enable)
		{
			if (ang > last_ang)
			{
				ShadowsFilter(tmp_filter, tmp_ang, sfp, tmp_ang);
				tmp.insert(tmp.end(), tmp_filter.begin(), tmp_filter.end());
				tmp_ang.clear();
				tmp_filter.clear();
				tmp_ang.push_back(ang);
				tmp_filter.push_back(point);
			}
			else
			{
				tmp_ang.push_back(ang);
				tmp_filter.push_back(point);
			}
			last_ang = ang;
		}
		else
		{
			tmp.push_back(point);
		}
	}
	for (unsigned int i = 0; i < tmp.size(); i++)
	{
		cloud_data.push_back(tmp.at(i));
	}
#if DEBUG_TEST
	m_filter_num += tmp.size();
#endif
	// std::cout << __LINE__ <<" "<<packet->dot_num<<" "<<tmp.size()<<" "<<tmp_filter.size()<<std::endl;
}

void PaceCatLidarSDK::UDPThreadProc(int id)
{
	RunConfig *cfg = GetConfig(id);
	int fd = SystemAPI::open_socket_port(cfg->listen_port, false);
	if (fd <= 0)
	{
		std::string err = "listen port open failed";
		WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
		return;
	}
	// 初始化打印SN以及版本号
	char cmd_result[128] = {0};
	int cmd_len = 0;
	if (CommunicationAPI::udp_talk_pack(fd, cfg->lidar_ip.c_str(), cfg->lidar_port, 6, "LUUIDH", C_PACK, cmd_len, cmd_result))
	{
		std::string err = "query sn:" + std::string(cmd_result, cmd_len);
		WriteLogData(cfg->ID, MSG_WARM, (char *)err.c_str(), err.size());
	}
	if (CommunicationAPI::udp_talk_pack(fd, cfg->lidar_ip.c_str(), cfg->lidar_port, 6, "LVERSH", C_PACK, cmd_len, cmd_result))
	{
		std::string err = "query version:" + std::string(cmd_result, cmd_len);
		WriteLogData(cfg->ID, MSG_WARM, (char *)err.c_str(), err.size());
	}
	// if (CommunicationAPI::udp_talk_pack(fd, cfg->lidar_ip.c_str(), cfg->lidar_port, 7, "LNETRRH", C_PACK, cmd_len, cmd_result))
	// {
	// 	std::string err = "network query:" + std::string(cmd_result, cmd_len);
	// 	WriteLogData(cfg->ID, MSG_WARM, (char *)err.c_str(), err.size());
	// }
	if (cfg->ptp_enable >= 0)
	{
		char cmd[16] = {0};
		char tmpresult[3] = {0};
		sprintf(cmd, "LSPTP:%cH", '0' + cfg->ptp_enable);
		// sprintf(cmd, "LSPTP:%dH", cfg->ptp_enable);
		if (CommunicationAPI::udp_talk_pack(fd, cfg->lidar_ip.c_str(), cfg->lidar_port, strlen(cmd), cmd, S_PACK, cfg->recv_len, tmpresult))
		{

			std::string err = "time: " + SystemAPI::getCurrentTime() + " set " + cmd + "  " + tmpresult;
			WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
		}
	}
	int16_t imu_packet_idx = -1;
	int16_t pointcloud_packet_idx = -1;
	uint64_t pointcloud_timestamp_last = 0; // 雷达最后一次更新时间戳
	uint64_t imu_timestamp_last = 0;		// imu最后一次更新时间戳

	uint16_t nlen = 0;
	uint8_t buffer[TRANS_BLOCK * 2];

	double last_ang = 100;
	std::vector<LidarCloudPointData> tmp_filter;
	std::vector<double> tmp_ang;
	int continuous_times = 0;
	uint64_t frame_starttime = 0;
	// save one frame pointcloud data
	LidarPacketData *pointclouddata = (LidarPacketData *)malloc(sizeof(LidarPacketData) + sizeof(LidarCloudPointData) * cfg->frame_package_num * 128);
	// save one packet imudata
	LidarPacketData *imudata = (LidarPacketData *)malloc(sizeof(LidarPacketData) + sizeof(LidarImuPointData));
	struct timeval to = {0, 10};
	while (cfg->run_state != QUIT)
	{
		if (cfg->run_state == OFFLINE)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}
		uint8_t buf[4096];
		sockaddr_in addr;
		socklen_t sz = sizeof(addr);
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(fd, &fds);
		int ret = select(fd + 1, &fds, NULL, NULL, &to);
		if (ret < 0)
			break;
		else if (ret == 0)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		else if (ret > 0)
		{
			if (FD_ISSET(fd, &fds))
			{
				int dw = recvfrom(fd, (char *)&buf, sizeof(buf), 0, (struct sockaddr *)&addr, &sz);
				if (dw > 0 && (strcmp((char *)inet_ntoa(addr.sin_addr), cfg->lidar_ip.c_str()) == 0))
				{
					if (buf[0] == 0 || buf[0] == 1)
					{
						// 判定是否是数据包
						const BlueSeaLidarEthernetPacket *packet = (BlueSeaLidarEthernetPacket *)&buf;
						if (pointcloud_timestamp_last)
						{
							int32_t diff = int32_t(packet->timestamp - pointcloud_timestamp_last);
							// printf("1:%d\n",diff);
							if (fabs(diff) > POINTCLOUD_TIMESTAMP_MAX_DIFF)
							{
								std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud packet interval large:" + std::to_string(diff) + "  " + std::to_string(POINTCLOUD_TIMESTAMP_MAX_DIFF);
								WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
							}
							if (diff < 0)
							{
								std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud packet time jump back:" + std::to_string(packet->timestamp) + "  " + std::to_string(pointcloud_timestamp_last);
								WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
							}
						}
						pointcloud_timestamp_last = packet->timestamp;

						int packet_size = sizeof(BlueSeaLidarEthernetPacket) + packet->dot_num * sizeof(BlueSeaLidarSpherPoint);
						if (dw != packet_size)
						{
							std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud packet length error: " + std::to_string(dw) + "  " + std::to_string(packet_size);
							WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
						}
						if (packet->udp_cnt != (uint8_t)pointcloud_packet_idx && pointcloud_packet_idx >= 0)
						{
							std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud packet lost :last " + std::to_string((uint8_t)pointcloud_packet_idx) + " now: " + std::to_string(packet->udp_cnt);
							WriteLogData(cfg->ID, MSG_WARM, (char *)err.c_str(), err.size());
						}
						pointcloud_packet_idx = packet->udp_cnt + 1;

						if (packet->version == 1)
						{
							if (packet->rt_v1.tags & TAG_MIRROR_NOT_STABLE)
							{
								std::string err = "time: " + SystemAPI::getCurrentTime() + " mirror " + std::to_string(packet->rt_v1.mirror_rpm);
								WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
								continue;
							}

							if (packet->rt_v1.tags & TAG_MOTOR_NOT_STABLE)
							{
								std::string err = "time: " + SystemAPI::getCurrentTime() + " main motor " + std::to_string(packet->rt_v1.motor_rpm_x10 / 10.0);
								WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
								continue;
							}
						}

						int count = cfg->cloud_data.size();
						if (count == 0)
						{
							cfg->frame_firstpoint_timestamp = packet->timestamp;
							if (cfg->timemode == 1)
							{
								std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
								std::chrono::nanoseconds nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch());
								frame_starttime = nanoseconds.count();
							}
							else
							{
								frame_starttime = cfg->frame_firstpoint_timestamp;
							}
						}
						AddPacketToList(packet, cfg->cloud_data, cfg->frame_firstpoint_timestamp, last_ang, tmp_filter, tmp_ang, cfg->sfp, cfg->mr_2);
						count = cfg->cloud_data.size();
						m_package_num++;
						if (cfg->frame_package_num < 80)
						{
							std::string err = "time: " + SystemAPI::getCurrentTime() + "too few point in one frame " + std::to_string(cfg->frame_package_num);
							WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
						}
						// if (count >= cfg->frame_package_num * 128)
						if (m_package_num >= cfg->frame_package_num)
						{
							// LidarPacketData *dat = (LidarPacketData *)malloc(sizeof(LidarPacketData) + sizeof(LidarCloudPointData) * count);
							LidarCloudPointData *dat2 = (LidarCloudPointData *)pointclouddata->data;
							struct timeval tv;
							SystemAPI::GetTimeStamp(&tv, true);

							int dirt_flag = 0;
							int point_idx = 0;
							// std::cout << __LINE__ <<" "<<count<<std::endl;
							for (int i = 0; i < count; i++)
							{
								if (isBitSet(cfg->cloud_data[i].tag, 7))
									dirt_flag++;

								if (!isBitSet(cfg->cloud_data[i].tag, 6))
								{
									dat2[point_idx] = cfg->cloud_data[i];
									point_idx++;
								}
							}
							pointclouddata->timestamp = frame_starttime;
							pointclouddata->length = sizeof(LidarPacketData) + sizeof(LidarCloudPointData) * point_idx;
							pointclouddata->dot_num = point_idx;
							pointclouddata->frame_cnt = cfg->frame_cnt++;
							pointclouddata->data_type = LIDARPOINTCLOUD;

							if (cfg->dfp.dfp_enable)
							{
								if (dirt_flag > point_idx * cfg->dfp.dirty_factor)
								{
									continuous_times++;
									if (continuous_times > cfg->dfp.continuous_times)
									{
										std::string err = "time: " + SystemAPI::getCurrentTime() + "Perhaps there is dirt or obstruction on the optical cover ! The tag points num:" + std::to_string(dirt_flag);
										WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
									}
								}
								else
								{
									continuous_times = 0;
								}
							}
							WritePointCloud(cfg->ID, 0, pointclouddata);
							memset(pointclouddata, 0, sizeof(LidarPacketData) + sizeof(LidarCloudPointData) * cfg->frame_package_num * 128);
							cfg->cloud_data.clear();
							// std::cout<<point_idx<<std::endl;

#if DEBUG_TEST
							float tmp_zero_point_num_factor = 1.0 * m_zero_point_num / m_sum_point_num;
							if (tmp_zero_point_num_factor > ZERO_POINT_FACTOR)
							{
								std::string err = "time: " + SystemAPI::getCurrentTime() + "too many zero point num, scale factor over " + std::to_string(ZERO_POINT_FACTOR) + "value:" + std::to_string(tmp_zero_point_num_factor);
								WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
							}
							float tmp_distance_close_num_factor = 1.0 * m_distance_close_num / m_sum_point_num;
							if (tmp_distance_close_num_factor > DISTANCE_CLOSE_FACTOR)
							{
								std::string err = "time: " + SystemAPI::getCurrentTime() + "too many  distance close num,scale factor over " + std::to_string(DISTANCE_CLOSE_FACTOR) + "value:" + std::to_string(tmp_distance_close_num_factor);
								WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
							}
							float tmp_sum_point_num_factor = 1.0 * m_filter_num / m_sum_point_num;
							if (tmp_sum_point_num_factor < SUM_POINT_NUM_FACTOR)
							{
								std::string err = "time: " + SystemAPI::getCurrentTime() + "too less sum point num,scale factor over " + std::to_string(SUM_POINT_NUM_FACTOR) + "value:" + std::to_string(tmp_sum_point_num_factor);
								WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
							}
							m_zero_point_num = 0;
							m_distance_close_num = 0;
							m_sum_point_num = 0;
							m_filter_num = 0;
#endif
							m_package_num = 0;
						}
					}
					else if (buf[0] == 0xfa && buf[1] == 0x88)
					{
						const TransBuf *trans = (TransBuf *)buf;

						if (trans->idx != (uint16_t)imu_packet_idx && imu_packet_idx >= 0)
						{
							std::string err = "time: " + SystemAPI::getCurrentTime() + " imudata packet lost :last " + std::to_string(trans->idx) + " now: " + std::to_string((uint16_t)imu_packet_idx);
							WriteLogData(cfg->ID, MSG_WARM, (char *)err.c_str(), err.size());
						}

						memcpy(buffer + nlen, trans->data, trans->len);
						nlen += trans->len;
						imu_packet_idx = trans->idx + 1;

						uint16_t n = Decode(nlen, buffer, cfg->imu_data /*, cfg->imu_mutex*/);
						for (int i = 0; i < nlen - n; i++)
						{
							buffer[i] = buffer[n + i];
						}
						nlen -= n;
						if (cfg->imu_data.size() > 0)
						{

							IIM42652_FIFO_PACKET_16_ST imu_stmp = cfg->imu_data.front();
							cfg->imu_data.pop();

							if (imu_timestamp_last)
							{
								int32_t diff = int32_t(imu_stmp.timestamp - imu_timestamp_last);
								// printf("2:%d\n",diff);
								if (fabs(diff) > IMU_TIMESTAMP_MAX_DIFF)
								{
									std::string err = "time: " + SystemAPI::getCurrentTime() + " imu packet interval large:" + std::to_string(diff) + "  " + std::to_string(IMU_TIMESTAMP_MAX_DIFF);
									WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
								}
								if (diff < 0)
								{
									std::string err = "time: " + SystemAPI::getCurrentTime() + " imu packet time jump back:" + std::to_string(imu_stmp.timestamp) + "  " + std::to_string(imu_timestamp_last);
									WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
								}
							}
							imu_timestamp_last = imu_stmp.timestamp;

							// LidarPacketData *dat = (LidarPacketData *)malloc(sizeof(LidarPacketData) + sizeof(LidarImuPointData));
							LidarImuPointData *dat2 = (LidarImuPointData *)imudata->data;

							dat2->gyro_x = imu_stmp.Gyro_X * 4000.0 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[0];
							dat2->gyro_y = imu_stmp.Gyro_Y * 4000.0 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[1];
							dat2->gyro_z = imu_stmp.Gyro_Z * 4000.0 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[2];
							dat2->acc_x = (imu_stmp.Accel_X * 4.0 / 0x10000);
							dat2->acc_y = (imu_stmp.Accel_Y * 4.0 / 0x10000);
							dat2->acc_z = (imu_stmp.Accel_Z * 4.0 / 0x10000);
							// dat2->acc_x = (imu_stmp.Accel_X * 4.0 / 0x10000) * cfg->imu_drift.K[0] + cfg->imu_drift.B[0];
							// dat2->acc_y = (imu_stmp.Accel_Y * 4.0 / 0x10000)* cfg->imu_drift.K[1] + cfg->imu_drift.B[1];
							// dat2->acc_z = (imu_stmp.Accel_Z * 4.0 / 0x10000)* cfg->imu_drift.K[2] + cfg->imu_drift.B[2];
							dat2->linear_acceleration_x = dat2->acc_x * cfg->imu_drift.R[0][0] + dat2->acc_y * cfg->imu_drift.R[0][1] + dat2->acc_z * cfg->imu_drift.R[0][2];
							dat2->linear_acceleration_y = dat2->acc_x * cfg->imu_drift.R[1][0] + dat2->acc_y * cfg->imu_drift.R[1][1] + dat2->acc_z * cfg->imu_drift.R[1][2];
							dat2->linear_acceleration_z = dat2->acc_x * cfg->imu_drift.R[2][0] + dat2->acc_y * cfg->imu_drift.R[2][1] + dat2->acc_z * cfg->imu_drift.R[2][2];
							if (cfg->timemode == 1)
								imudata->timestamp = getCurrentNanoseconds();
							else
								imudata->timestamp = imu_stmp.timestamp;

							imudata->length = sizeof(LidarPacketData) + sizeof(LidarImuPointData);
							imudata->dot_num = 1;
							imudata->frame_cnt = cfg->frame_cnt;
							imudata->data_type = LIDARIMUDATA;
							WriteImuData(cfg->ID, 0, imudata);
							memset(imudata, 0, sizeof(sizeof(LidarPacketData) + sizeof(LidarImuPointData)));
						}
					}
					else if (buf[0] == 0xfa && buf[1] == 0x89) // debug
					{
					}
					else if (buf[0] == 0x4c && buf[1] == 0x48 && (unsigned char)buf[2] == 0xbe && (unsigned char)buf[3] == 0xb4) // time sync
					{
					}
					else if (buf[0] == 0x4c && buf[1] == 0x48 && (unsigned char)buf[2] == 0xac && (unsigned char)buf[3] == 0xb8) // dev_param
					{
					}
					else if (buf[0] == 0x4c && buf[1] == 0x4d && (unsigned char)buf[2] == 0x53 && (unsigned char)buf[3] == 0x47) // alarm
					{
						// LidarMsgHdr* hdr = (LidarMsgHdr*)(buf);
					}
					else
					{
						const uint8_t *ptr = (uint8_t *)&buf;
						printf("%d : %02x %02x %02x %02x %02x %02x %02x %02x\n",
							   dw,
							   ptr[0], ptr[1], ptr[2], ptr[3],
							   ptr[4], ptr[5], ptr[6], ptr[7]);
					}
					if (pointcloud_timestamp_last && imu_timestamp_last)
					{
						int32_t diff = int32_t(pointcloud_timestamp_last - imu_timestamp_last);
						// printf("3:%d\n",diff);
						if (fabs(diff) > POINTCLOUD_IMU_TIMESTAMP_MAX_DIFF)
						{
							std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud and imu packet timestamp not sync" + std::to_string(diff) + "  " + std::to_string(POINTCLOUD_IMU_TIMESTAMP_MAX_DIFF);
							WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
						}
					}
				}
			}
		}
		switch (cfg->action)
		{
		case NONE:
		case FINISH:
		{
			break;
		}
		case START:
		case STOP:
		{
			char tmpresult[64] = {0};
			if (!CommunicationAPI::udp_talk_pack(fd, cfg->lidar_ip.c_str(), cfg->lidar_port, cfg->send_len, cfg->send_buf.c_str(), C_PACK, cfg->recv_len, tmpresult))
			{
				cfg->recv_buf = "NG";
			}
			cfg->recv_buf = tmpresult;
			cfg->recv_len = cfg->recv_buf.size();
			cfg->action = FINISH;
			break;
		}
		case RESTART:
		{
			CommunicationAPI::send_cmd_udp(fd, cfg->lidar_ip.c_str(), cfg->lidar_port, C_PACK, rand(), cfg->send_len, cfg->send_buf.c_str());
			cfg->recv_buf = "OK";
			cfg->recv_len = 2;
			cfg->action = FINISH;
			break;
		}
		case GET_PARAMS:
		{
			char tmpresult[512] = {0};
			if (!CommunicationAPI::udp_talk_pack(fd, cfg->lidar_ip.c_str(), cfg->lidar_port, cfg->send_len, cfg->send_buf.c_str(), GS_PACK, cfg->recv_len, tmpresult))
			{
				cfg->recv_buf = "NG";
				cfg->recv_len = 2;
			}
			cfg->recv_buf = std::string(tmpresult, cfg->recv_len);
			cfg->action = FINISH;
			break;
		}
		case GET_VERSION:
		{
			char tmpresult[128] = {0};
			if (!CommunicationAPI::udp_talk_pack(fd, cfg->lidar_ip.c_str(), cfg->lidar_port, cfg->send_len, cfg->send_buf.c_str(), C_PACK, cfg->recv_len, tmpresult))
			{
				cfg->recv_buf = "NG";
				cfg->recv_len = 2;
			}
			cfg->recv_buf = tmpresult;
			cfg->recv_len = cfg->recv_buf.size();
			cfg->action = FINISH;
			break;
		}
		case SET_NETWORK:
		{
			CommunicationAPI::send_cmd_udp(fd, cfg->lidar_ip.c_str(), cfg->lidar_port, 0x0053, rand(), cfg->send_len, cfg->send_buf.c_str());
			cfg->recv_buf = "OK";
			cfg->recv_len = 2;
			cfg->action = FINISH;
			break;
		}
		case SET_UPLOAD_NETWORK:
		{
			if (!CommunicationAPI::udp_talk_pack(fd, cfg->lidar_ip.c_str(), cfg->lidar_port, cfg->send_len, cfg->send_buf.c_str(), S_PACK, cfg->recv_len, (char *)cfg->recv_buf.c_str()))
			{
				cfg->recv_buf = "NG";
				cfg->recv_len = 2;
			}
			cfg->action = FINISH;
			break;
		}
		case SET_UPLOAD_FIX:
		{
			char tmpresult[64] = {0};
			if (!CommunicationAPI::udp_talk_pack(fd, cfg->lidar_ip.c_str(), cfg->lidar_port, cfg->send_len, cfg->send_buf.c_str(), S_PACK, cfg->recv_len, tmpresult))
			{
				cfg->recv_buf = "NG";
				cfg->recv_len = 2;
			}
			cfg->recv_buf = tmpresult;
			cfg->recv_len = cfg->recv_buf.size();
			cfg->action = FINISH;
			break;
		}
		case UPGRADE:
		{
			bool ret = PaceCatLidarSDK::getInstance()->FirmwareUpgrade(cfg->lidar_ip.c_str(), cfg->lidar_port, cfg->send_buf, cfg->recv_buf);
			cfg->recv_len = cfg->recv_buf.size();
			if (ret)
			{
				cfg->recv_buf = "OK";
				cfg->recv_len = 2;
			}
			cfg->action = FINISH;
			break;
		}
		case PTP_INIT:
		{
			char tmpresult[64] = {0};
			if (!CommunicationAPI::udp_talk_pack(fd, cfg->lidar_ip.c_str(), cfg->lidar_port, cfg->send_len, cfg->send_buf.c_str(), C_PACK, cfg->recv_len, tmpresult))
			{
				cfg->recv_buf = "NG";
			}
			cfg->recv_buf = tmpresult;
			cfg->recv_len = cfg->recv_buf.size();
			cfg->action = FINISH;
			break;
		}
		case GET_NETERR:
		{
			char tmpresult[64] = {0};
			if (!CommunicationAPI::udp_talk_pack(fd, cfg->lidar_ip.c_str(), cfg->lidar_port, cfg->send_len, cfg->send_buf.c_str(), C_PACK, cfg->recv_len, tmpresult))
			{
				cfg->recv_buf = "NG";
			}
			cfg->recv_buf = tmpresult;
			cfg->recv_len = cfg->recv_buf.size();
			cfg->action = FINISH;
			break;
		}
		case CACHE_CLEAR:
		{
			cfg->cloud_data.clear();
			m_package_num=0;
			pointcloud_timestamp_last=0;
			imu_timestamp_last=0;
			cfg->recv_buf = "OK";
			cfg->recv_len = cfg->recv_buf.size();
			cfg->action = FINISH;
			break;
		}
		}
	}
	SystemAPI::closefd(fd, true);
	free(pointclouddata);
	free(imudata);
	std::string err = "thread sub end";
	WriteLogData(cfg->ID, MSG_DEBUG, (char *)err.c_str(), err.size());
}

// void PubCloudThreadProc(int id)
// {
// 	RunConfig *cfg = PaceCatLidarSDK::getInstance()->GetConfig(id);

// 	while (cfg->run_state != QUIT)
// 	{
// 		if (cfg->run_state == OFFLINE)
// 		{
// 			std::this_thread::sleep_for(std::chrono::milliseconds(100));
// 			continue;
// 		}
// 		cfg->data_mutex.lock();
// 		int count = cfg->cloud_data.size();
// 		if (count >= ONE_FRAME_POINT_NUM)
// 		{
// 			LidarPacketData *dat = (LidarPacketData *)malloc(sizeof(LidarPacketData) + sizeof(LidarCloudPointData) * count);
// 			LidarCloudPointData *dat2 = (LidarCloudPointData *)dat->data;
// 			struct timeval tv;
// 			SystemAPI::GetTimeStamp(&tv, true);

// 			for (int i = 0; i < count; i++)
// 			{
// 				dat2[i] = cfg->cloud_data[i];
// 			}

// 			dat->timestamp = cfg->frame_firstpoint_timestamp;
// 			dat->length = sizeof(LidarPacketData) + sizeof(LidarCloudPointData) * count;
// 			dat->dot_num = count;
// 			dat->frame_cnt = cfg->frame_cnt++;
// 			dat->data_type = LIDARPOINTCLOUD;

// 			PaceCatLidarSDK::getInstance()->WritePointCloud(cfg->ID, 0, dat);
// 			cfg->cloud_data.clear();

// 			if (cfg->cloud_data_cache.size() >= 0)
// 			{
// 				cfg->cloud_data = cfg->cloud_data_cache;
// 				cfg->frame_firstpoint_timestamp = cfg->frame_firstpoint_timestamp_cache;
// 				cfg->cloud_data_cache.clear();
// 			}
// 		}
// 		cfg->data_mutex.unlock();

// 		if (count < ONE_FRAME_POINT_NUM)
// 			std::this_thread::sleep_for(std::chrono::milliseconds(3));
// 	}
// }

// void PubImuThreadProc(int id)
// {
// 	RunConfig *cfg = PaceCatLidarSDK::getInstance()->GetConfig(id);
// 	while (cfg->run_state != QUIT)
// 	{
// 		if (cfg->run_state == OFFLINE)
// 		{
// 			std::this_thread::sleep_for(std::chrono::milliseconds(100));
// 			continue;
// 		}
// 		if (!cfg->imu_data.empty())
// 		{

// 			// printf("imu queue %ld\n", cfg->imu_data.size());
// 			cfg->imu_mutex.lock();
// 			IIM42652_FIFO_PACKET_16_ST imu_stmp = cfg->imu_data.front();
// 			cfg->imu_data.pop();
// 			cfg->imu_mutex.unlock();
// 			LidarPacketData *dat = (LidarPacketData *)malloc(sizeof(LidarPacketData) + sizeof(LidarImuPointData));
// 			LidarImuPointData *dat2 = (LidarImuPointData *)dat->data;

// 			dat2->gyro_x = imu_stmp.Gyro_X * 4000.0 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[0];
// 			dat2->gyro_y = imu_stmp.Gyro_Y * 4000.0 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[1];
// 			dat2->gyro_z = imu_stmp.Gyro_Z * 4000.0 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[2];
// 			dat2->acc_x = (imu_stmp.Accel_X * 4.0 / 0x10000);
// 			dat2->acc_y = (imu_stmp.Accel_Y * 4.0 / 0x10000);
// 			dat2->acc_z = (imu_stmp.Accel_Z * 4.0 / 0x10000);
// 			// dat2->acc_x = (imu_stmp.Accel_X * 4.0 / 0x10000) * cfg->imu_drift.K[0] + cfg->imu_drift.B[0];
// 			// dat2->acc_y = (imu_stmp.Accel_Y * 4.0 / 0x10000)* cfg->imu_drift.K[1] + cfg->imu_drift.B[1];
// 			// dat2->acc_z = (imu_stmp.Accel_Z * 4.0 / 0x10000)* cfg->imu_drift.K[2] + cfg->imu_drift.B[2];
// 			dat2->linear_acceleration_x = dat2->acc_x * cfg->imu_drift.R[0][0] + dat2->acc_y * cfg->imu_drift.R[0][1] + dat2->acc_z * cfg->imu_drift.R[0][2];
// 			dat2->linear_acceleration_y = dat2->acc_x * cfg->imu_drift.R[1][0] + dat2->acc_y * cfg->imu_drift.R[1][1] + dat2->acc_z * cfg->imu_drift.R[1][2];
// 			dat2->linear_acceleration_z = dat2->acc_x * cfg->imu_drift.R[2][0] + dat2->acc_y * cfg->imu_drift.R[2][1] + dat2->acc_z * cfg->imu_drift.R[2][2];
// 			dat->timestamp = imu_stmp.timestamp;
// 			dat->length = sizeof(LidarPacketData) + sizeof(LidarImuPointData);
// 			dat->dot_num = 1;
// 			dat->frame_cnt = cfg->frame_cnt;
// 			dat->data_type = LIDARIMUDATA;
// 			PaceCatLidarSDK::getInstance()->WriteImuData(cfg->ID, 0, dat);

// 			// printf("%f  %f  %lf\n",dat2->gyro_x,dat2->linear_acceleration_x,cfg->imu_drift.Gyro[0]);
// 			// std::this_thread::sleep_for(std::chrono::milliseconds(5));
// 		}
// 		else
// 			std::this_thread::sleep_for(std::chrono::milliseconds(10));
// 	}
// }

uint16_t Decode(uint16_t n, const uint8_t *buf, std::queue<IIM42652_FIFO_PACKET_16_ST> &imu_data /*, std::mutex &imu_mutex*/)
{
	uint16_t idx = 0;
	int ndrop = 0;
	uint8_t drop[1024];
	while (idx < n)
	{
		uint8_t ch = buf[idx];

		if (ch == 0xdd)
		{
			if (idx + 4 > n)
				break;
			idx += 4;
		}
		else if (ch == 0xdc || ch == 0xde)
		{
			if (idx + 14 > n)
				break;
			uint16_t v = buf[idx + 4];
			v = (v << 8) + buf[idx + 5];
			uint32_t ts[2];
			memcpy(ts, buf + idx + 6, 8);
			idx += 14;
		}
		else if (ch == 0xd3)
		{
			if (idx + 25 > n)
				break;

			IIM42652_FIFO_PACKET_16_ST *imu = (IIM42652_FIFO_PACKET_16_ST *)(buf + idx + 1);
			// imu_mutex.lock();
			imu_data.push(*imu);
			// imu_mutex.unlock();
			idx += 25;
		}
		else if (ch == 0xee)
		{
			if (idx + 6 > n)
				break;
			idx += 6;
		}
		else
		{
			drop[ndrop++] = ch;
			idx++;
		}
	}
	if (ndrop > 0)
	{
		printf("drop %d : %02x %02x %02x %02x %02x %02x\n",
			   ndrop,
			   drop[0], drop[1], drop[2],
			   drop[3], drop[4], drop[5]);
	}

	return idx;
}

int PaceCatLidarSDK::PackNetCmd(uint16_t type, uint16_t len, uint16_t sn, const void *buf, uint8_t *netbuf)
{
	CmdHeader *hdr = (CmdHeader *)netbuf;

	hdr->sign = PACK_PREAMLE;
	hdr->cmd = type; // S_PACK;
	hdr->sn = sn;	 // rand();

	hdr->len = len;
	int len4 = ((len + 3) >> 2) * 4;
	if (len > 0)
	{
		memcpy(netbuf + sizeof(CmdHeader), buf, len);
	}

	// int n = sizeof(CmdHeader);
	uint32_t *pcrc = (uint32_t *)(netbuf + sizeof(CmdHeader) + len4);
	pcrc[0] = stm32crc((uint32_t *)(netbuf + 0), len4 / 4 + 2);

	return len4 + 12;
}

int PaceCatLidarSDK::SendNetPack(int sock, uint16_t type, uint16_t len, const void *buf, char *ip, int port)
{
	uint8_t netbuf[1024];
	int netlen = PackNetCmd(type, len, rand(), buf, netbuf);

	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr(ip);
	addr.sin_port = htons(port);

	return sendto(sock, (char *)netbuf, netlen, 0,
				  (struct sockaddr *)&addr, sizeof(struct sockaddr));
}

int PaceCatLidarSDK::read_calib(const char *lidar_ip, int port)
{
#ifdef _WIN32
	WSADATA wsda; //   Structure   to   store   info
	WSAStartup(MAKEWORD(2, 2), &wsda);
#endif // _WIN32
	int sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	printf("read data from %s:%d\n", lidar_ip, port);
	uint8_t zbuf[16] = {0};
	SendNetPack(sockfd, DRIFT_RD_PACK, 4, zbuf, (char *)lidar_ip, port);

	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(sockfd, &readfds);

	struct timeval tv;
	tv.tv_sec = 5;
	tv.tv_usec = 0;
	int retval = select(sockfd + 1, &readfds, NULL, NULL, &tv);
	if (retval < 0)
	{
		printf("sock err\n");
		close(sockfd);
		return retval;
	}
	if (retval == 0)
	{
		close(sockfd);
		printf("sock timeout\n");
		return -1;
	}

	char buf[1024] = {0};
	sockaddr_in addr;
	socklen_t sz = sizeof(addr);
	recvfrom(sockfd, (char *)&buf, sizeof(buf), 0,
			 (struct sockaddr *)&addr, &sz);

	// close(sockfd);
	const CmdHeader *hdr = (CmdHeader *)buf;
	if (hdr->sign != PACK_PREAMLE)
	{
		printf("unknown response\n");
		return -1;
	}

	uint32_t len4 = ((hdr->len + 3) >> 2) * 4;
	const uint32_t *pcrc = (uint32_t *)((char *)buf + sizeof(CmdHeader) + len4);
	uint32_t chk = stm32crc((uint32_t *)buf, len4 / 4 + 2);

	if (*pcrc != chk)
	{
		printf("crc error\n");
		return -1;
	}

	uint16_t type = ~(hdr->cmd);
	if (type != DRIFT_RD_PACK || hdr->len < sizeof(DriftCalib))
	{
		printf("not drift data\n");
		return -1;
	}

	DriftCalib drift;
	memcpy(&drift, buf + sizeof(CmdHeader), sizeof(drift));
	if (drift.code != DRIFT_MAGIC)
	{
		printf("bad drift data\n");
		return -1;
	}
	int id = QueryIDByIp(lidar_ip);
	RunConfig *cfg = GetConfig(id);
	cfg->imu_drift = drift.drifts.imu;
	printf("imu : %f, %f, %f; %f, %f, %f; %f, %f, %f \n",
		   cfg->imu_drift.R[0][0], cfg->imu_drift.R[0][1], cfg->imu_drift.R[0][2],
		   cfg->imu_drift.R[1][0], cfg->imu_drift.R[1][1], cfg->imu_drift.R[1][2],
		   cfg->imu_drift.R[2][0], cfg->imu_drift.R[2][1], cfg->imu_drift.R[2][2]);

	printf("gyro : %f, %f, %f\n",
		   cfg->imu_drift.Gyro[0], cfg->imu_drift.Gyro[1], cfg->imu_drift.Gyro[2]);

	return 0;
}

void PaceCatLidarSDK::HeartThreadProc(HeartInfo &heartinfo)
{
#ifdef _WIN32
	WSADATA wsda; //   Structure   to   store   info
	WSAStartup(MAKEWORD(2, 2), &wsda);
#endif
	int sock = socket(AF_INET, SOCK_DGRAM, 0);
	int yes = 1;
	if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char *)&yes, sizeof(yes)) < 0)
	{
		heartinfo.value = "socket init error";
		heartinfo.code = SystemAPI::getLastError();
		SystemAPI::closefd(sock, true);
		return;
	}

	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(HEARTPORT);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	int iResult = ::bind(sock, (struct sockaddr *)&addr, sizeof(addr));
	if (iResult != 0)
	{
		heartinfo.value = "bind port failed";
		heartinfo.code = SystemAPI::getLastError();
		SystemAPI::closefd(sock, true);
		return;
	}

	struct ip_mreq mreq;
	mreq.imr_multiaddr.s_addr = inet_addr("225.225.225.225");
	mreq.imr_interface.s_addr = htonl(INADDR_ANY);
	if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&mreq, sizeof(mreq)) < 0)
	{
		heartinfo.value = "socket init error";
		heartinfo.code = SystemAPI::getLastError();
		SystemAPI::closefd(sock, true);
		return;
	}

	struct timeval tv;
	SystemAPI::GetTimeStamp(&tv, false);
	time_t tto = tv.tv_sec + 1;
	socklen_t sz = sizeof(addr);
	while (heartinfo.isrun)
	{
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(sock, &fds);
		struct timeval to = {1, 0};
		int ret = select(sock + 1, &fds, NULL, NULL, &to);
		if (ret > 0)
		{
			char raw[4096];
			int dw = recvfrom(sock, raw, sizeof(raw), 0, (struct sockaddr *)&addr, &sz);
			if (dw == sizeof(DevHeart))
			{
				DevHeart *devheart = (DevHeart *)raw;
				std::string sn = BaseAPI::stringfilter(devheart->dev_sn, 20);
				char tmp_ip[16] = {0};
				sprintf(tmp_ip, "%d.%d.%d.%d", devheart->ip[0], devheart->ip[1], devheart->ip[2], devheart->ip[3]);
				std::string ip = BaseAPI::stringfilter(tmp_ip, 16);
				int id = PaceCatLidarSDK::getInstance()->QueryIDByIp(ip);
				bool isexist = false;
				for (unsigned int i = 0; i < heartinfo.lidars.size(); i++)
				{
					if (sn == heartinfo.lidars[i].sn)
					{
						heartinfo.lidars[i].timestamp = devheart->timestamp[1] * 1000 + devheart->timestamp[0] / 1000000;
						heartinfo.lidars[i].motor_rpm = devheart->motor_rpm;
						heartinfo.lidars[i].mirror_rpm = devheart->mirror_rpm;
						heartinfo.lidars[i].temperature = devheart->temperature;
						heartinfo.lidars[i].voltage = devheart->voltage;

						isexist = true;
						heartinfo.lidars[i].flag = true;
						if (!heartinfo.lidars[i].isonline)
						{
							heartinfo.lidars[i].isonline = true;

							std::string result = sn + " " + ip + "  online";
							WriteLogData(id, 0, (char *)result.c_str(), result.size());
						}
						break;
					}
					// exist  two more lidars with same ip
					if (ip == heartinfo.lidars[i].ip)
					{
						heartinfo.value = "multiple lidars with the same ip";
						heartinfo.code = -1;
						// heartinfo.isrun=false;
						break;
					}
				}
				if (!isexist)
				{
					ConnectInfo info;
					info.ip = ip;
					info.sn = sn;
					info.port = devheart->port;
					info.timestamp = (devheart->timestamp[0]) * 1000 + devheart->timestamp[1] / 1000000;
					info.temperature = devheart->temperature;
					info.motor_rpm = devheart->motor_rpm;
					info.mirror_rpm = devheart->mirror_rpm;
					info.temperature = devheart->temperature;
					info.voltage = devheart->voltage;
					info.isonline = true;
					info.flag = true;
					heartinfo.lidars.push_back(info);
					std::string result = sn + " " + ip + "  online";
					WriteLogData(id, 0, (char *)result.c_str(), result.size());
				}
			}
		}

		// check is outtime
		SystemAPI::GetTimeStamp(&tv, false);
		if (tv.tv_sec > tto)
		{
			for (unsigned int i = 0; i < heartinfo.lidars.size(); i++)
			{
				if (heartinfo.lidars[i].isonline == ONLINE && !heartinfo.lidars[i].flag)
				{
					int id = QueryIDByIp(heartinfo.lidars[i].ip);
					heartinfo.lidars[i].isonline = false;
					heartinfo.lidars[i].motor_rpm = 0;
					heartinfo.lidars[i].mirror_rpm = 0;
					heartinfo.lidars[i].temperature = 0;
					heartinfo.lidars[i].voltage = 0;
					std::string result = heartinfo.lidars[i].sn + " " + heartinfo.lidars[i].ip + "  offline";
					WriteLogData(id, 0, (char *)result.c_str(), result.size());
				}
				heartinfo.lidars[i].flag = false;
			}
			tto = tv.tv_sec + 1;
		}
	}
	SystemAPI::closefd(sock, true);
}

bool PaceCatLidarSDK::FirmwareUpgrade(std::string ip, int port, std::string path, std::string &error)
{
	int id = QueryIDByIp(ip);
	if (id < 0)
		return false;
	std::string result = "firmware update start";
	PaceCatLidarSDK::getInstance()->WriteLogData(id, 0, (char *)result.c_str(), result.size());

	int udp = SystemAPI::open_socket_port(port, true);
	if (udp <= 0)
	{
		result = "open listen port failed: " + std::to_string(port);
		PaceCatLidarSDK::getInstance()->WriteLogData(id, 0, (char *)result.c_str(), result.size());
		return false;
	}
	unsigned short rands = rand();
	ResendPack resndBuf;
	FirmwareFile *firmwareFile = LoadFirmware(path.c_str());
	FirmwarePart fp;
	if (!firmwareFile)
	{
		result = "Failed to load firmware upgrade file: " + std::to_string(port);
		PaceCatLidarSDK::getInstance()->WriteLogData(id, 0, (char *)result.c_str(), result.size());
		SystemAPI::closefd(udp, true);
		return false;
	}
	result = "load firmware file ok";
	PaceCatLidarSDK::getInstance()->WriteLogData(id, 0, (char *)result.c_str(), result.size());

	FirmWriteResp *resp = NULL;
	bool isBreak = false;
	fp.offset = OP_FLASH_ERASE;
	fp.buf[0] = firmwareFile->len;
	fp.buf[1] = firmwareFile->crc;
	memset(&resndBuf, 0, sizeof(ResendPack));
	int idx = 0;
	SendUpgradePack(udp, &fp, (char *)ip.c_str(), port, rands, &resndBuf);
	result = "send flash erase";
	PaceCatLidarSDK::getInstance()->WriteLogData(id, 0, (char *)result.c_str(), result.size());
	while (!isBreak)
	{
		if (resp != NULL)
		{
			delete resp;
			resp = NULL;
		}
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(udp, &fds);
		struct timeval to = {5, 0};
		int ret = select(udp + 1, &fds, NULL, NULL, &to);
		if (ret < 0)
		{
			continue;
		}
		else if (ret == 0)
			continue;

		if (resndBuf.timeout < time(NULL))
		{
			SendUpgradePack(udp, &fp, (char *)ip.c_str(), port, rands, &resndBuf);
			idx++;
			if (idx > 3)
			{
				// int offset=((FirmwarePart*)(resndBuf.buf))->offset;

				result = "send cmd more than three times without answering";
				PaceCatLidarSDK::getInstance()->WriteLogData(id, 0, (char *)result.c_str(), result.size());
				idx = 0;
				break;
			}
		}
		if (FD_ISSET(udp, &fds))
		{
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);

			char buf[1024] = {0};
			int nr = recvfrom(udp, buf, sizeof(buf), 0, (struct sockaddr *)&addr, &sz);
			if (nr > 0)
			{
				CmdHeader *hdr = (CmdHeader *)buf;
				if (hdr->sign != 0x484c)
					continue;

				if (hdr->sn == rands)
				{
					if (resndBuf.sn == hdr->sn)
					{
						memset(&resndBuf, 0, sizeof(ResendPack));
					}
					resp = new FirmWriteResp;
					memcpy((void *)resp, buf + sizeof(CmdHeader), hdr->len);
					resp->msg[sizeof(resp->msg) - 1] = 0;
					if (resp->result != 0)
						break;
					idx = 0;
					if (resp->offset == OP_WRITE_IAP)
					{
						result = "receive OP_WRITE_IAP";
						PaceCatLidarSDK::getInstance()->WriteLogData(id, 0, (char *)result.c_str(), result.size());
						if (resp->result == 0)
						{
							FirmwarePart fp;
							// 重启机器
							fp.offset = OP_FRIMWARE_RESET;
							fp.buf[0] = 0xabcd1234;
							SendUpgradePack(udp, &fp, (char *)ip.c_str(), port, rands, &resndBuf);
							SystemAPI::closefd(udp, true);
							result = "firmware file update success";
							PaceCatLidarSDK::getInstance()->WriteLogData(id, 0, (char *)result.c_str(), result.size());
							return true;
						}
						else
						{
							error = resp->result;
							SystemAPI::closefd(udp, true);
							result = "firmware file update failed:" + error;
							PaceCatLidarSDK::getInstance()->WriteLogData(id, 0, (char *)result.c_str(), result.size());

							return false;
						}
					}
					if (firmwareFile != NULL)
					{
						if (resp->offset == OP_FLASH_ERASE)
						{
							fp.offset = 0;
							result = "receive OP_FLASH_ERASE";
							PaceCatLidarSDK::getInstance()->WriteLogData(id, 0, (char *)result.c_str(), result.size());
						}
						else
						{
							// printf("%x\n",fp.offset);
							fp.offset = resp->offset + 512;
						}

						if (fp.offset + 512 <= (uint32_t)firmwareFile->len)
						{
							result = "offset:" + std::to_string(fp.offset) + "total:" + std::to_string(firmwareFile->len);
							PaceCatLidarSDK::getInstance()->WriteLogData(id, 0, (char *)result.c_str(), result.size());

							memcpy(fp.buf, firmwareFile->buffer + fp.offset, 512);
							fp.crc = stm32crc(fp.buf, 128);
						}
						else
						{
							// MYLOG<<"OP_WRITE_IAP";
							fp.offset = OP_WRITE_IAP;
							fp.buf[0] = firmwareFile->len;
							fp.buf[1] = firmwareFile->crc;
							result = "send OP_WRITE_IAP";
							PaceCatLidarSDK::getInstance()->WriteLogData(id, 0, (char *)result.c_str(), result.size());
						}
						SendUpgradePack(udp, &fp, (char *)ip.c_str(), port, rands, &resndBuf);
					}
				}
			}
		}
	}
	SystemAPI::closefd(udp, true);
	return false;
}
FirmwareFile *LoadFirmware(const char *path)
{
	FILE *pfile = nullptr;
	long length = 0; // 升级文件长度
	pfile = fopen(path, "rb");
	if (pfile == NULL)
	{
		printf("open error\n");
		return NULL;
	}
	else
	{
		fseek(pfile, 0, SEEK_END);
		length = ftell(pfile);
		fseek(pfile, 0, SEEK_SET);
	}
	FirmwareFile *fp = NULL;
	fp = (FirmwareFile *)new char[length];
	fread(fp, length, 1, pfile);
	fclose(pfile);
	// printf("%d %d  %d  %d\n",__LINE__,fp->len,sizeof(FirmwareFile),length);

	if ((unsigned int)fp->code == 0xb18e03ea && fp->len % 512 == 0 && fp->len + sizeof(FirmwareFile) == (unsigned long)length)
	{
		if (fp->crc == stm32crc((uint32_t *)(fp->buffer), fp->len / 4))
		{

			return fp;
		}
	}
	return NULL;
}

void SendUpgradePack(unsigned int udp, const FirmwarePart *fp, char *ip, int port, int SN, ResendPack *resndBuf)
{
	CommunicationAPI::send_cmd_udp(udp, ip, port, F_PACK, SN, sizeof(FirmwarePart), (char *)fp);
	if (resndBuf)
	{
		resndBuf->cmd = F_PACK;
		resndBuf->len = sizeof(FirmwarePart);
		resndBuf->tried = 1;
		resndBuf->sn = SN;
		resndBuf->timeout = time(NULL) + TIMEOUT;
		memcpy(resndBuf->buf, fp, sizeof(FirmwarePart));
	}
}
double getAngleWithViewpoint(float r1, float r2, double included_angle)
{
	return atan2(r2 * sinf(included_angle), r1 - (r2 * cosf(included_angle)));
}
int ShadowsFilter(std::vector<LidarCloudPointData> &scan_in, std::vector<double> &ang_in, const ShadowsFilterParam &param, std::vector<double> &tmp_ang)
{
	// double angle_increment = 0;
	std::set<int> indices_to_delete;
	for (unsigned int i = 0; i < scan_in.size() - param.window - 1; i++)
	{
		double dis_i = sqrt((scan_in[i].x * scan_in[i].x) + (scan_in[i].y * scan_in[i].y) + (scan_in[i].z * scan_in[i].z));
		if ((dis_i > param.effective_distance) | (dis_i == 0))
			continue;
		for (int y = 1; y < param.window + 1; y++)
		{
			int j = i + y;
			// double dis_i = sqrt((scan_in[i].x * scan_in[i].x) + (scan_in[i].y * scan_in[i].y) + (scan_in[i].z * scan_in[i].z));
			double dis_j = sqrt((scan_in[j].x * scan_in[j].x) + (scan_in[j].y * scan_in[j].y) + (scan_in[j].z * scan_in[j].z));
			// //屏蔽某些特殊的平面(水平面)
			// double    dis = sqrt(pow(scan_in[i].x - scan_in[j].x, 2) + pow(scan_in[i].y - scan_in[j].y, 2) + pow(scan_in[i].z - scan_in[j].z, 2));
			// double    z = fabs(scan_in[i].z - scan_in[j].z);
			// double    result = sqrt(dis*dis - z*z);
			// double   ang_final = atan2(result,z);
			// ang_final = abs(ang_final * 180 /M_PI);
			// if(ang_final < 10 || ang_final > 80) continue;
			if (j < 0 || j >= (int)scan_in.size() || (int)i == j)
				continue;
			// if(fabs(dis_i- dis_j) < 0.05)
			// continue;
			double rad = getAngleWithViewpoint(
				dis_i,
				dis_j,
				tmp_ang[i] - tmp_ang[j]);

			double angle = abs(rad * 180 / M_PI);
			// std::cout << angle<< std::endl;
			if (angle > param.max_angle || angle < param.min_angle)
			{
				// std::cout << ang_in[i]-ang_in[j]<< std::endl;
				int from, to;
				// if (dis_i < dis_j)
				{
					from = i + 1;
					to = j;
				}
				// else
				// {
				// from = j - 1;
				// to = i;
				// }

				if (from > to)
				{
					int t = from;
					from = to;
					to = t;
				}
				for (int index = from; index <= to; index++)
				{
					indices_to_delete.insert(index);
				}
			}
		}
	}

	int nr = 0;
	for (std::set<int>::iterator it = indices_to_delete.begin(); it != indices_to_delete.end(); ++it)
	{
		scan_in[*it].tag += 64;
		nr++;
	}

	return nr;
}

bool isBitSet(uint8_t num, int n)
{
	return (num & (1 << n)) != 0; // 左移 n 位并与 num 按位与
}
