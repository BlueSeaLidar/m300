// M300_SDK.cpp: 定义应用程序的入口点。
//
#include <fstream>
#include <chrono>
#include <iostream>
#include <chrono>
#include <ctime>
#include <sstream>
#include "global.h"
#include "pacecatlidarsdk.h"
#include "playback.h"
#include "upgrade.h"
using namespace moodycamel;
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
	m_heartinfo.isrun = false;
}

PaceCatLidarSDK::~PaceCatLidarSDK()
{
}
bool PaceCatLidarSDK::SetPointCloudCallback(int ID, LidarCloudPointCallback cb, void *client_data)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->cb_cloudpoint = cb;
	lidar->cloudpoint = client_data;
	return true;
}
bool PaceCatLidarSDK::SetImuDataCallback(int ID, LidarImuDataCallback cb, void *client_data)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->cb_imudata = cb;
	lidar->imudata = (char *)client_data;
	return true;
}
bool PaceCatLidarSDK::SetLogDataCallback(int ID, LidarLogDataCallback cb, void *client_data)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->cb_logdata = cb;
	lidar->logdata = (char *)client_data;
	return true;
}
bool PaceCatLidarSDK::SetAlarmDataCallback(int ID, LidarAlarmCallback cb, void *client_data)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->cb_alarmdata = cb;
	lidar->alarmdata = (char *)client_data;
	return true;
}

void PaceCatLidarSDK::WritePointCloud(int ID, const uint8_t dev_type, LidarPacketData *data)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return;

	if (lidar->cb_cloudpoint != nullptr)
		lidar->cb_cloudpoint(ID, dev_type, data, lidar->cloudpoint);
}

void PaceCatLidarSDK::WriteImuData(int ID, const uint8_t dev_type, LidarPacketData *data)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return;
	if (lidar->cb_imudata != nullptr)
		lidar->cb_imudata(ID, dev_type, data, lidar->imudata);
}

void PaceCatLidarSDK::WriteLogData(int ID, const uint8_t dev_type, char *data, int len)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return;

	if (lidar->cb_logdata != nullptr)
	{
		char logbuf[1024] = {0};
		sprintf(logbuf, "time:%s %s", SystemAPI::getCurrentTime().c_str(), data);
		lidar->cb_logdata(ID, dev_type, logbuf, strlen(logbuf));
	}
	// lidar->cb_logdata(ID, dev_type, "1234567890qwertyuiopasdfghjklzxcvbnm1234567890qwertyuiopasdfghjklzxcvbnm1234567890qwertyuiopasdfghjklzxcvbnm",108);
}
void PaceCatLidarSDK::WriteAlarmData(int ID, const uint8_t dev_type, char *data, int len)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return;

	if (lidar->cb_alarmdata != nullptr)
		lidar->cb_alarmdata(ID, dev_type, data, len);
	// lidar->cb_alarmdata(ID, dev_type, "qwertyuiopasdfghjklzxcvbnm1234567890qwertyuiopasdfghjklzxcvbnm1234567890qwertyuiopasdfghjklzxcvbnm1234567890",108);
	//
}
void PaceCatLidarSDK::Init(std::string adapter)
{
	// Conflict with ros's single-process framework, temporarily disabled
	if (!m_heartinfo.isrun)
	{
		m_heartinfo.code = 0;
		m_heartinfo.isrun = true;
		m_heartinfo.adapter = adapter;
		m_heartthread = std::thread(&PaceCatLidarSDK::HeartThreadProc, PaceCatLidarSDK::getInstance(), std::ref(m_heartinfo));
		m_heartthread.detach();
	}
}
void PaceCatLidarSDK::Uninit()
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		m_lidars.at(i)->run_state = QUIT;
	}
	m_heartinfo.isrun = false;
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
int PaceCatLidarSDK::AddLidarForPlayback(std::string logpath, int frame_rate)
{
	RunConfig *cfg = new RunConfig;
	cfg->log_path = logpath;
	cfg->frame_package_num = frame_rate;
	cfg->ID = m_lidars.size();
	cfg->run_state = OFFLINE;
	m_lidars.push_back(cfg);
	return cfg->ID;
}
int PaceCatLidarSDK::AddLidarForUpgrade(std::string lidar_ip, int lidar_port, int listen_port)
{
	RunConfig *cfg = new RunConfig;
	cfg->lidar_ip = lidar_ip;
	cfg->lidar_port = lidar_port;
	cfg->listen_port = listen_port;
	cfg->ID = m_lidars.size();
	cfg->run_state = OFFLINE;
	m_lidars.push_back(cfg);
	return cfg->ID;
}
bool PaceCatLidarSDK::ConnectLidar(int ID, bool isplayback)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	if (!isplayback)
	{
		// 读取标定文件
		bool result = false;
		for (int i = 5; i >= 0; i--)
		{
			result = PaceCatLidarSDK::getInstance()->ReadCalib(ID, lidar->lidar_ip, lidar->lidar_port);
			if (result)
				break;
		}
		char log_buf[128] = {0};
		sprintf(log_buf, "read calib result:%s", result ? RECV_OK : RECV_NG);
		WriteLogData(ID, MSG_WARM, log_buf, strlen(log_buf));
		if (!result)
			return false;

		lidar->thread_data = std::thread(&PaceCatLidarSDK::UDPDataThreadProc, PaceCatLidarSDK::getInstance(), ID);
		lidar->thread_data.detach();
		lidar->thread_cmd = std::thread(&PaceCatLidarSDK::UDPCmdThreadProc, PaceCatLidarSDK::getInstance(), ID);
		lidar->thread_cmd.detach();
	}
	else
	{
		std::thread parselog_thread = std::thread(&PaceCatLidarSDK::ParseLogThreadProc, PaceCatLidarSDK::getInstance(), ID);
		parselog_thread.detach();

		std::thread playback_thread = std::thread(&PaceCatLidarSDK::PlaybackThreadProc, PaceCatLidarSDK::getInstance(), ID);
		playback_thread.detach();
	}
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
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->send_len = 6;
	strcpy(lidar->send_buf, "xxxxxx");
	lidar->send_type = GS_PACK;
	lidar->action = LidarAction::CMD_TALK;
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
			EEpromV101 *eepromv101 = (EEpromV101 *)lidar->recv_buf;
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
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	strcpy(lidar->send_buf, "LXVERH");
	lidar->send_len = 6;
	lidar->send_type = C_PACK;
	lidar->action = LidarAction::CMD_TALK;
	int index = CMD_REPEAT;
	while (lidar->action != FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == FINISH)
	{
		lidar->action = NONE;
		if (strcmp(lidar->recv_buf, RECV_NG) == 0)
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
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return UserHeartInfo{0, 0, 0, 0, 0, 0, 0, ""};

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
			heartinfo.timestamp = m_heartinfo.lidars[i].timestamp;
			heartinfo.code = m_heartinfo.code;
			heartinfo.value = m_heartinfo.value;
			return heartinfo;
		}
	}
	return UserHeartInfo{0, 0, 0, 0, 0, 0, m_heartinfo.code, m_heartinfo.value};
}

bool PaceCatLidarSDK::SetLidarNetWork(int ID, std::string ip, std::string mask, std::string gateway, uint16_t port)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
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
	memcpy(lidar->send_buf, tmp, lidar->send_len);
	lidar->send_type = S_PACK;
	lidar->action = LidarAction::CMD_TALK;

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
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
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
	memcpy(lidar->send_buf, tmp, lidar->send_len);
	lidar->send_type = S_PACK;
	lidar->action = LidarAction::CMD_TALK;

	int index = CMD_REPEAT;
	while (lidar->action != FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == FINISH)
	{
		lidar->action = NONE;
		if (strcmp(lidar->recv_buf, RECV_OK) == 0)
			return true;
	}
	return false;
}

bool PaceCatLidarSDK::SetLidarAction(int ID, int action)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->send_len = 6;
	if (action == START)
		strcpy(lidar->send_buf, "LSTARH");
	else if (action == STOP)
		strcpy(lidar->send_buf, "LSTOPH");
	else if (action == RESTART)
		strcpy(lidar->send_buf, "LRESTH");
	else
		return false;

	lidar->send_type = C_PACK;
	lidar->action = LidarAction::CMD_TALK;
	int index = CMD_REPEAT;
	while (lidar->action != FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == FINISH)
	{
		lidar->action = NONE;
		if (strcmp(lidar->recv_buf, RECV_OK) == 0)
			return true;
	}

	return false;
}

bool PaceCatLidarSDK::SetLidarUpgrade(int ID, std::string path)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	std::ifstream f(path.c_str());
	if (!f.good())
		return false;

	return PaceCatLidarSDK::getInstance()->FirmwareUpgrade(ID, lidar->lidar_ip.c_str(), lidar->lidar_port, lidar->listen_port, path);
}

bool PaceCatLidarSDK::SetLidarPTPInit(int ID)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	char tmp[64] = {0};
	sprintf(tmp, "LPTPINITH");
	lidar->send_len = strlen(tmp);
	memcpy(lidar->send_buf, tmp, lidar->send_len);
	lidar->send_type = C_PACK;
	lidar->action = LidarAction::CMD_TALK;
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
		std::string recv_buf = lidar->recv_buf;
		if (recv_buf.find("PTP sync") != std::string::npos)
			return true;
	}

	return false;
}
bool PaceCatLidarSDK::QueryLidarNetWork(int ID, std::string &netinfo)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	char tmp[64] = {0};
	sprintf(tmp, "LNETRRH");
	lidar->send_len = 7;
	memcpy(lidar->send_buf, tmp, lidar->send_len);
	lidar->send_type = C_PACK;
	lidar->action = LidarAction::CMD_TALK;
	int index = CMD_REPEAT;
	while (lidar->action != FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == FINISH)
	{
		lidar->action = NONE;
		std::string recv_buf = lidar->recv_buf;
		if (recv_buf.find("Boot:") != std::string::npos)
		{
			// printf("%s %d\n",lidar->recv_buf.c_str(),lidar->recv_len);
			netinfo = std::string(lidar->recv_buf, 0, lidar->recv_len);
			return true;
		}
	}

	return false;
}
void PaceCatLidarSDK::ClearFrameCache(int ID)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return;

	lidar->cache_clear = true;
}

bool PaceCatLidarSDK::QueryDirtyData(int ID, std::string &dirty_data)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->action = LidarAction::CMD_TALK;
	lidar->send_type = C_PACK;
	lidar->send_len = 6;
	strcpy(lidar->send_buf, "LURERH");
	int index = CMD_REPEAT;
	while (lidar->action != FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == FINISH)
	{
		lidar->action = NONE;
		dirty_data = std::string(lidar->recv_buf, lidar->recv_len);
		return true;
	}
	return false;
}
bool PaceCatLidarSDK::QueryMCUInfo(int ID, std::string &mcu_data)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	strcpy(lidar->send_buf, "LQLVXH");
	lidar->send_len = 6;
	lidar->action = LidarAction::CMD_TALK;
	lidar->send_type = C_PACK;
	int index = CMD_REPEAT;
	while (lidar->action != FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == FINISH)
	{
		lidar->action = NONE;
		mcu_data = std::string(lidar->recv_buf, lidar->recv_len);
		return true;
	}
	return false;
}

bool PaceCatLidarSDK::QueryLidarErrList(int ID, std::string &errlist)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	lidar->action = LidarAction::CMD_TALK;
	lidar->send_type = C_PACK;
	strcpy(lidar->send_buf, "LEVENTRH");
	lidar->send_len = 8;
	int index = CMD_REPEAT;
	while (lidar->action != FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == FINISH)
	{
		lidar->action = NONE;
		errlist = std::string(lidar->recv_buf, lidar->recv_len);
		return true;
	}
	return false;
}
bool PaceCatLidarSDK::CleanLidarErrList(int ID)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	strcpy(lidar->send_buf, "LEVENTCH");
	lidar->send_len = 8;
	lidar->action = LidarAction::CMD_TALK;
	lidar->send_type = C_PACK;
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
bool PaceCatLidarSDK::QueryRainData(int ID, uint8_t &rain)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	rain = lidar->rain;
	return true;
}
bool PaceCatLidarSDK::QueryEchoMode(int ID, uint8_t &echo_mode)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	echo_mode = lidar->echo_mode;
	return true;
}

bool PaceCatLidarSDK::QueryADCInfo(int ID, std::string &adcinfo)
{
	RunConfig *lidar = GetConfig(ID);
	if (lidar == nullptr)
		return false;

	strcpy(lidar->send_buf, "LMBADCH");
	lidar->send_len = 7;
	lidar->action = LidarAction::CMD_TALK;
	lidar->send_type = C_PACK;
	int index = CMD_REPEAT;
	while (lidar->action != FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}
	if (lidar->action == FINISH)
	{
		lidar->action = NONE;
		adcinfo = std::string(lidar->recv_buf, lidar->recv_len);
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

	if (point.x == 0 && point.y == 0 && point.z == 0)
		point.reflectivity = 0;

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

void PaceCatLidarSDK::UDPDataThreadProc(int id)
{
	RunConfig *cfg = GetConfig(id);
	if (cfg == nullptr)
		return;
	unsigned char recv_data_buf[4096] = {0};
	char log_buf[1024] = {0};

	// 判定传入的包数量是否不符合规范[120-200]
	if (cfg->frame_package_num <= 120 || cfg->frame_package_num >= 200)
	{
		sprintf(log_buf, "one frame package num is out of range,thread end");
		WriteLogData(cfg->ID, MSG_ERROR, log_buf, strlen(log_buf));
		return;
	}

	int fd = SystemAPI::open_socket_port(cfg->listen_port, false);
	if (fd <= 0)
	{
		std::string err = "listen port open failed";
		WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
		return;
	}
	// 清空异常列表
	DeBugInfo debuginfo;
	memset(&debuginfo, 0, sizeof(DeBugInfo));
	debuginfo.imu_packet_idx = -1;
	debuginfo.pointcloud_packet_idx = -1;
	debuginfo.timer = LOG_TIMER * 1000000000ULL; // 定时检测时间

	// uint16_t nlen = 0;
	// uint8_t buffer[TRANS_BLOCK * 2];
	double last_ang = 100;
	std::vector<LidarCloudPointData> tmp_filter;
	std::vector<double> tmp_ang;
	int continuous_times = 0;
	uint64_t frame_starttime = 0;
	sockaddr_in addr;
	socklen_t sz = sizeof(addr);

	// save one frame pointcloud data
	LidarPacketData *pointclouddata = (LidarPacketData *)malloc(sizeof(LidarPacketData) + sizeof(LidarCloudPointData) * cfg->frame_package_num * 128);
	// save one packet imudata
	LidarPacketData *imudata = (LidarPacketData *)malloc(sizeof(LidarPacketData) + sizeof(LidarImuPointData));
	struct timeval to = {0, 100};

	while (cfg->run_state != QUIT)
	{
		if (cfg->cache_clear)
		{
			cfg->cloud_data.clear();
			cfg->package_num_idx = 0;
			debuginfo.pointcloud_timestamp_last = 0;
			debuginfo.imu_timestamp_last = 0;
			cfg->cache_clear = false;
		}
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(fd, &fds);
		int ret = select(fd + 1, &fds, NULL, NULL, &to);
		if (ret < 0)
			break;
		else if (ret == 0)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			continue;
		}
		else if (ret > 0)
		{
			if (FD_ISSET(fd, &fds))
			{
				int dw = recvfrom(fd, (char *)&recv_data_buf, sizeof(recv_data_buf), 0, (struct sockaddr *)&addr, &sz);
				if (dw > 0 && (strcmp((char *)inet_ntoa(addr.sin_addr), cfg->lidar_ip.c_str()) == 0))
				{
					if (recv_data_buf[0] == 0 || recv_data_buf[0] == 1)
					{
						// 判定是否是数据包
						const BlueSeaLidarEthernetPacket *packet = (BlueSeaLidarEthernetPacket *)&recv_data_buf;
						if (debuginfo.pointcloud_timestamp_last)
						{
							int64_t diff = int64_t(packet->timestamp - debuginfo.pointcloud_timestamp_last);
							if (diff > POINTCLOUD_TIMESTAMP_MAX_DIFF)
							{
								std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud packet interval large:" + std::to_string(diff) + "  " + std::to_string(POINTCLOUD_TIMESTAMP_MAX_DIFF);

								WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
							}
							if (diff < 0)
							{
								std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud packet time jump back:" + std::to_string(packet->timestamp) + "  " + std::to_string(debuginfo.pointcloud_timestamp_last);
								WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
							}
						}
						debuginfo.pointcloud_timestamp_last = packet->timestamp;
						int packet_size = sizeof(BlueSeaLidarEthernetPacket) + packet->dot_num * sizeof(BlueSeaLidarSpherPoint);
						if (dw != packet_size)
						{
							std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud packet length error: " + std::to_string(dw) + "  " + std::to_string(packet_size);
							WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
						}
						if (packet->udp_cnt != (uint8_t)debuginfo.pointcloud_packet_idx && debuginfo.pointcloud_packet_idx >= 0)
						{
							std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud packet lost :last " + std::to_string((uint8_t)debuginfo.pointcloud_packet_idx) + " now: " + std::to_string(packet->udp_cnt) + "index:" + std::to_string(debuginfo.pointcloud_timestamp_drop_num);
							WriteLogData(cfg->ID, MSG_WARM, (char *)err.c_str(), err.size());
						}
						debuginfo.pointcloud_packet_idx = packet->udp_cnt + 1;
						if (packet->version == 1)
						{
							// 从数据包的tag标签判定下雨系数以及单双回波 0单回波 1双回波
							if (packet->rt_v1.tag & TAG_WITH_RAIN_DETECT)
								cfg->rain = packet->rt_v1.rain;

							if (packet->rt_v1.tag & TAG_DUAL_ECHO_MODE)
								cfg->echo_mode = 1;
							else
								cfg->echo_mode = 0;

							if (packet->rt_v1.tag & TAG_MIRROR_NOT_STABLE)
							{
								debuginfo.mirror_err_timestamp_num++;
								if (packet->timestamp - debuginfo.mirror_err_timestamp_last > debuginfo.timer)
								{
									std::string err = "time: " + SystemAPI::getCurrentTime() + " mirror " + std::to_string(packet->rt_v1.mirror_rpm) + " index:" + std::to_string(debuginfo.mirror_err_timestamp_num);
									WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
									debuginfo.mirror_err_timestamp_num = 0;
									debuginfo.mirror_err_timestamp_last = packet->timestamp;

									uint32_t error_code = 0;
									setbit(error_code, ERR_MIRROR_NO_STABLE);
									PaceCatLidarSDK::getInstance()->WriteAlarmData(cfg->ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
								}
								continue;
							}
							else
							{
								if ((packet->timestamp - debuginfo.mirror_err_timestamp_last > debuginfo.timer) && (debuginfo.mirror_err_timestamp_num > 0))
								{
									std::string err = "time: " + SystemAPI::getCurrentTime() + " mirror " + std::to_string(packet->rt_v1.mirror_rpm) + " index:" + std::to_string(debuginfo.mirror_err_timestamp_num);
									WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
									debuginfo.mirror_err_timestamp_num = 0;
									debuginfo.mirror_err_timestamp_last = packet->timestamp;

									uint32_t error_code = 0;
									setbit(error_code, ERR_MIRROR_NO_STABLE);
									PaceCatLidarSDK::getInstance()->WriteAlarmData(cfg->ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
								}
							}

							if (packet->rt_v1.tag & TAG_MOTOR_NOT_STABLE)
							{
								debuginfo.motor_err_timestamp_num++;
								if (packet->timestamp - debuginfo.motor_err_timestamp_last > debuginfo.timer)
								{
									std::string err = "time: " + SystemAPI::getCurrentTime() + " main motor " + std::to_string(packet->rt_v1.motor_rpm_x10 / 10.0) + " index:" + std::to_string(debuginfo.motor_err_timestamp_num);
									WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());

									debuginfo.motor_err_timestamp_num = 0;
									debuginfo.motor_err_timestamp_last = packet->timestamp;

									uint32_t error_code = 0;
									setbit(error_code, ERR_MOTOR_NO_STABLE);
									PaceCatLidarSDK::getInstance()->WriteAlarmData(cfg->ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
								}
								continue;
							}
							else
							{
								if ((packet->timestamp - debuginfo.motor_err_timestamp_last > debuginfo.timer) && debuginfo.motor_err_timestamp_num > 0)
								{
									std::string err = "time: " + SystemAPI::getCurrentTime() + " main motor " + std::to_string(packet->rt_v1.motor_rpm_x10 / 10.0) + " index:" + std::to_string(debuginfo.motor_err_timestamp_num);
									WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());

									debuginfo.motor_err_timestamp_num = 0;
									debuginfo.motor_err_timestamp_last = packet->timestamp;

									uint32_t error_code = 0;
									setbit(error_code, ERR_MOTOR_NO_STABLE);
									PaceCatLidarSDK::getInstance()->WriteAlarmData(cfg->ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
								}
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
						// printf("%d %s %d\n", __LINE__, __FUNCTION__,id);
						// fflush(stdout);

						AddPacketToList(packet, cfg->cloud_data, cfg->frame_firstpoint_timestamp, last_ang, tmp_filter, tmp_ang, cfg->sfp, cfg->mr_2);
						count = cfg->cloud_data.size();
						cfg->package_num_idx++;
						if (cfg->package_num_idx >= cfg->frame_package_num)
						{
							LidarCloudPointData *dat2 = (LidarCloudPointData *)pointclouddata->data;
							SystemAPI::GetTimeStamp(true);

							int dirt_flag = 0;
							int point_idx = 0;
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

#if DEBUG_TEST
							float tmp_zero_point_num_factor = 1.0 * m_zero_point_num / m_sum_point_num;
							if (tmp_zero_point_num_factor > ZERO_POINT_FACTOR)
							{
								debuginfo.zero_pointdata_timestamp_num++;
								if (packet->timestamp - debuginfo.zero_pointdata_timestamp_last > debuginfo.timer)
								{
									std::string err = "time: " + SystemAPI::getCurrentTime() + "too many zero point num, scale factor over " + std::to_string(ZERO_POINT_FACTOR) + "value:" + std::to_string(tmp_zero_point_num_factor) + "index:" + std::to_string(debuginfo.zero_pointdata_timestamp_num);
									WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());

									uint32_t error_code = 0;
									setbit(error_code, ERR_DATA_ZERO);
									PaceCatLidarSDK::getInstance()->WriteAlarmData(cfg->ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
									debuginfo.zero_pointdata_timestamp_num = 0;
									debuginfo.zero_pointdata_timestamp_last = packet->timestamp;
								}
							}
							else
							{
								if ((packet->timestamp - debuginfo.zero_pointdata_timestamp_last > debuginfo.timer) && (debuginfo.zero_pointdata_timestamp_num > 0))
								{
									std::string err = "time: " + SystemAPI::getCurrentTime() + "too many zero point num, scale factor over " + std::to_string(ZERO_POINT_FACTOR) + "value:" + std::to_string(tmp_zero_point_num_factor) + "index:" + std::to_string(debuginfo.zero_pointdata_timestamp_num);
									WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());

									uint32_t error_code = 0;
									setbit(error_code, ERR_DATA_ZERO);
									PaceCatLidarSDK::getInstance()->WriteAlarmData(cfg->ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
									debuginfo.zero_pointdata_timestamp_num = 0;
									debuginfo.zero_pointdata_timestamp_last = packet->timestamp;
								}
							}
							float tmp_distance_close_num_factor = 1.0 * m_distance_close_num / m_sum_point_num;
							if (tmp_distance_close_num_factor > DISTANCE_CLOSE_FACTOR)
							{
								debuginfo.distance_close_timestamp_num++;
								if (packet->timestamp - debuginfo.distance_close_timestamp_last > debuginfo.timer)
								{
									std::string err = "time: " + SystemAPI::getCurrentTime() + "too many  distance close num,scale factor over " + std::to_string(DISTANCE_CLOSE_FACTOR) + "value:" + std::to_string(tmp_distance_close_num_factor) + "index:" + std::to_string(debuginfo.distance_close_timestamp_num);
									WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
									debuginfo.distance_close_timestamp_num = 0;
									debuginfo.distance_close_timestamp_last = packet->timestamp;
								}
							}
							else
							{
								if ((packet->timestamp - debuginfo.distance_close_timestamp_last > debuginfo.timer) && (debuginfo.distance_close_timestamp_num > 0))
								{
									std::string err = "time: " + SystemAPI::getCurrentTime() + "too many  distance close num,scale factor over " + std::to_string(DISTANCE_CLOSE_FACTOR) + "value:" + std::to_string(tmp_distance_close_num_factor) + "index:" + std::to_string(debuginfo.distance_close_timestamp_num);
									WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
									debuginfo.distance_close_timestamp_num = 0;
									debuginfo.distance_close_timestamp_last = packet->timestamp;
								}
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
							cfg->package_num_idx = 0;
						}
					}
					else if (recv_data_buf[0] == 0xfa && recv_data_buf[1] == 0x88)
					{
						TransBuf *trans = (TransBuf *)recv_data_buf;
						IIM42652_FIFO_PACKET_16_ST *imu_stmp = (IIM42652_FIFO_PACKET_16_ST *)(trans->data + 1);
						if (trans->idx != (uint16_t)debuginfo.imu_packet_idx && debuginfo.imu_packet_idx >= 0)
						{
							std::string err = "time: " + SystemAPI::getCurrentTime() + " imudata packet lost :last " + std::to_string((uint16_t)debuginfo.imu_packet_idx) + " now: " + std::to_string(trans->idx) + " index:" + std::to_string(debuginfo.imu_timestamp_drop_num);
							WriteLogData(cfg->ID, MSG_WARM, (char *)err.c_str(), err.size());
						}
						debuginfo.imu_packet_idx = trans->idx + 1;
						if (debuginfo.imu_timestamp_last)
						{
							int64_t diff = int64_t(imu_stmp->timestamp - debuginfo.imu_timestamp_last);
							// printf("2:%d\n",diff);
							if (fabs(diff) > IMU_TIMESTAMP_MAX_DIFF)
							{
								std::string err = "time: " + SystemAPI::getCurrentTime() + " imu packet interval large:" + std::to_string(diff) + "  " + std::to_string(IMU_TIMESTAMP_MAX_DIFF);
								WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
							}
							if (diff < 0)
							{
								std::string err = "time: " + SystemAPI::getCurrentTime() + " imu packet time jump back:" + std::to_string(imu_stmp->timestamp) + "  " + std::to_string(debuginfo.imu_timestamp_last);
								WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
							}
						}
						debuginfo.imu_timestamp_last = imu_stmp->timestamp;
						LidarImuPointData *dat2 = (LidarImuPointData *)imudata->data;

						dat2->gyro_x = imu_stmp->Gyro_X * 4000.0 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[0];
						dat2->gyro_y = imu_stmp->Gyro_Y * 4000.0 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[1];
						dat2->gyro_z = imu_stmp->Gyro_Z * 4000.0 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[2];
						dat2->acc_x = (imu_stmp->Accel_X * 4.0 / 0x10000);
						dat2->acc_y = (imu_stmp->Accel_Y * 4.0 / 0x10000);
						dat2->acc_z = (imu_stmp->Accel_Z * 4.0 / 0x10000);
						// dat2->acc_x = (imu_stmp.Accel_X * 4.0 / 0x10000) * cfg->imu_drift.K[0] + cfg->imu_drift.B[0];
						// dat2->acc_y = (imu_stmp.Accel_Y * 4.0 / 0x10000)* cfg->imu_drift.K[1] + cfg->imu_drift.B[1];
						// dat2->acc_z = (imu_stmp.Accel_Z * 4.0 / 0x10000)* cfg->imu_drift.K[2] + cfg->imu_drift.B[2];
						dat2->linear_acceleration_x = dat2->acc_x * cfg->imu_drift.R[0][0] + dat2->acc_y * cfg->imu_drift.R[0][1] + dat2->acc_z * cfg->imu_drift.R[0][2];
						dat2->linear_acceleration_y = dat2->acc_x * cfg->imu_drift.R[1][0] + dat2->acc_y * cfg->imu_drift.R[1][1] + dat2->acc_z * cfg->imu_drift.R[1][2];
						dat2->linear_acceleration_z = dat2->acc_x * cfg->imu_drift.R[2][0] + dat2->acc_y * cfg->imu_drift.R[2][1] + dat2->acc_z * cfg->imu_drift.R[2][2];
						if (cfg->timemode == 1)
							imudata->timestamp = getCurrentNanoseconds();
						else
							imudata->timestamp = imu_stmp->timestamp;

						imudata->length = sizeof(LidarPacketData) + sizeof(LidarImuPointData);
						imudata->dot_num = 1;
						imudata->frame_cnt = cfg->frame_cnt;
						imudata->data_type = LIDARIMUDATA;
						WriteImuData(cfg->ID, 0, imudata);
						memset(imudata, 0, sizeof(sizeof(LidarPacketData) + sizeof(LidarImuPointData)));
					}
					else if (recv_data_buf[0] == 0xfa && recv_data_buf[1] == 0x89) // debug
					{
					}
					else if (recv_data_buf[0] == 0x4c && recv_data_buf[1] == 0x4d && (unsigned char)recv_data_buf[2] == 0x53 && (unsigned char)recv_data_buf[3] == 0x47) // alarm
					{
						// LidarMsgHdr* hdr = (LidarMsgHdr*)(buf);
					}
					else
					{
						const uint8_t *ptr = (uint8_t *)&recv_data_buf;
						printf("%d : %02x %02x %02x %02x %02x %02x %02x %02x\n",
							   dw,
							   ptr[0], ptr[1], ptr[2], ptr[3],
							   ptr[4], ptr[5], ptr[6], ptr[7]);
					}
					if (debuginfo.pointcloud_timestamp_last && debuginfo.imu_timestamp_last)
					{
						int64_t diff = int64_t(debuginfo.pointcloud_timestamp_last - debuginfo.imu_timestamp_last);
						if (fabs(diff) > POINTCLOUD_IMU_TIMESTAMP_MAX_DIFF)
						{
							debuginfo.pointcloud_imu_timestamp_large_num++;
							uint64_t new_timestamp = SystemAPI::GetTimeStamp(true);
							if (new_timestamp - debuginfo.pointcloud_imu_timestamp_large_last > debuginfo.timer / 1000000)
							{
								std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud and imu packet timestamp not sync" + std::to_string(diff) + "  " + std::to_string(POINTCLOUD_IMU_TIMESTAMP_MAX_DIFF) + " index:" + std::to_string(debuginfo.pointcloud_imu_timestamp_large_num);
								WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
								debuginfo.pointcloud_imu_timestamp_large_num = 0;
								debuginfo.pointcloud_imu_timestamp_large_last = new_timestamp;
							}
						}
						else
						{
							uint64_t new_timestamp = SystemAPI::GetTimeStamp(true);
							if ((new_timestamp - debuginfo.pointcloud_imu_timestamp_large_last > debuginfo.timer / 1000000) && (debuginfo.pointcloud_imu_timestamp_large_num > 0))
							{
								std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud and imu packet timestamp not sync" + std::to_string(diff) + "  " + std::to_string(POINTCLOUD_IMU_TIMESTAMP_MAX_DIFF) + " index:" + std::to_string(debuginfo.pointcloud_imu_timestamp_large_num);
								WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
								debuginfo.pointcloud_imu_timestamp_large_num = 0;
								debuginfo.pointcloud_imu_timestamp_large_last = new_timestamp;
							}
						}
					}
				}
			}
		}
	}
	SystemAPI::closefd(fd, true);
	free(pointclouddata);
	free(imudata);
	std::string err = "recv and parse thread  end";
	WriteLogData(cfg->ID, MSG_DEBUG, (char *)err.c_str(), err.size());
}
void PaceCatLidarSDK::UDPCmdThreadProc(int id)
{
	RunConfig *cfg = GetConfig(id);
	if (cfg == nullptr)
		return;

	int cmdfd = SystemAPI::open_socket_port();
	if (cmdfd <= 0)
	{
		std::string err = "create cmd socket failed";
		WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
		return;
	}
	// 初始化指令队列
	CmdTaskList cmdtasklist;
	cmdtasklist.max_try_count = 5;
	cmdtasklist.max_waittime = 2;

	char log_buf[1024] = {0};	 // 日志打印缓存
	uint8_t cmd_buf[1024] = {0}; // 指令打印缓存
	struct timeval to = {0, 100};
	sockaddr_in addr;
	socklen_t sz = sizeof(addr);
	/// cmdtasklist.cmdtask.push(CmdTask{0, 0, "LUUIDH", C_PACK,(uint16_t)rand()});
	// cmdtasklist.cmdtask.push(CmdTask{0, 0, "LXVERH",C_PACK,(uint16_t)rand()});
	// cmdtasklist.cmdtask.push(CmdTask{0, 0, "XXXXXX",GS_PACK,(uint16_t)rand()});
	while (cfg->run_state != LidarState::QUIT)
	{
		// 任务分发
		if (cfg->action == LidarAction::CMD_TALK)
		{
			cmdtasklist.cmdtask.push(CmdTask{0, 0, cfg->send_buf, cfg->send_type});
			cfg->action = LidarAction::NONE;
		}
		// 指令任务处理
		if (!cmdtasklist.cmdtask.empty())
		{
			CmdTask &cmdtask = cmdtasklist.cmdtask.front();
			// 超过最大检测次数
			if (cmdtask.tried > cmdtasklist.max_try_count)
			{
				cmdtasklist.cmdtask.pop();
				continue;
			}
			uint64_t timestamp = SystemAPI::GetTimeStamp(true);
			// 如果发送时间为0或者与当前时间超过最大间隔重发
			if (cmdtask.send_timestamp == 0 || (timestamp - cmdtask.send_timestamp > cmdtasklist.max_waittime * 1000))
			{
				sprintf(log_buf, "cmddata:%s cmd_size:%lu cmd_type:%d time:%lu %lu %d\n", cmdtask.cmd.c_str(), cmdtask.cmd.size(), cmdtask.cmd_type, timestamp, cmdtask.send_timestamp, cmdtask.tried);
				WriteLogData(cfg->ID, MSG_ERROR, log_buf, strlen(log_buf));

				CommunicationAPI::send_cmd_udp(cmdfd, cfg->lidar_ip.c_str(), cfg->lidar_port, cmdtask.cmd_type, cmdtask.rand, cmdtask.cmd.size(), cmdtask.cmd.c_str());
				cmdtask.send_timestamp = timestamp;
				cmdtask.tried++;
			}
		}
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(cmdfd, &fds);
		int ret = select(cmdfd + 1, &fds, NULL, NULL, &to);
		if (ret < 0)
			break;
		else if (ret == 0)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}
		else if (ret > 0)
		{
			if (FD_ISSET(cmdfd, &fds))
			{
				int dw = recvfrom(cmdfd, (char *)&cmd_buf, sizeof(cmd_buf), 0, (struct sockaddr *)&addr, &sz);
				if (dw > 0 && (strcmp((char *)inet_ntoa(addr.sin_addr), cfg->lidar_ip.c_str()) == 0))
				{
					CmdHeader *cmdheader = (CmdHeader *)cmd_buf;
					uint16_t cmd = ~(cmdheader->cmd);
					while (1)
					{
						if (cmd == C_PACK || cmd == S_PACK || cmd == GS_PACK)
						{
							CmdTask &cmdtask = cmdtasklist.cmdtask.front();
							if (cmdtask.rand == cmdheader->sn)
							{
								if (strcmp(cmdtask.cmd.c_str(), "LEVENTRH") == 0)
								{
									std::string syslog = bin_to_hex_fast(cmd_buf + sizeof(CmdHeader), cmdheader->len);
									memcpy(cfg->recv_buf, syslog.c_str(), syslog.size());
									cfg->recv_len = syslog.size();
								}
								else
								{
									cfg->recv_len = cmdheader->len;
									memcpy(cfg->recv_buf, cmd_buf + sizeof(CmdHeader), cmdheader->len);
								}
								cfg->action = LidarAction::FINISH;
							}
							// 当出现随机码不一致时，说明有指令无应答，可能是硬件被重新上下电，意外操作等,尝试对
							else
							{
								sprintf(log_buf, "Command mismatch. Trying next command.");
								WriteLogData(cfg->ID, MSG_ERROR, log_buf, strlen(log_buf));
							}
							cmdtasklist.cmdtask.pop();
							if (cmdtasklist.cmdtask.size() == 0)
								break;
						}
						else
						{
							sprintf(log_buf, "find unknown cmd!");
							WriteLogData(cfg->ID, MSG_ERROR, log_buf, strlen(log_buf));
							break;
						}
					}
				}
			}
		}
	}
}

void PaceCatLidarSDK::ParseLogThreadProc(int id)
{
	RunConfig *cfg = GetConfig(id);
	if (cfg == nullptr)
		return;

	ParseContext ctx = {0};
	printf("logpath:%s \n", cfg->log_path.c_str());
	// 初始化上下文
	if (init_context(&ctx, cfg->log_path.c_str()) != 0)
	{
		std::string err = "init_context error";
		WriteLogData(cfg->ID, MSG_DEBUG, (char *)err.c_str(), err.size());
		return;
	}
	std::string err = "log file size:" + std::to_string((double)ctx.file_size / (1024 * 1024 * 1024)) + "GB";
	WriteLogData(cfg->ID, MSG_DEBUG, (char *)err.c_str(), err.size());
	ctx.running = 1;
	while (ctx.running && ctx.processed_size < ctx.file_size)
	{
		if (map_next_chunk(&ctx) != 0)
		{
			break;
		}
		const uint8_t *data = (const uint8_t *)ctx.mmap_ptr;
		size_t remaining = ctx.mmap_size - ctx.current_offset;

		// 如果是文件开头，跳过PCAP全局头
		if (ctx.processed_size == 0)
		{
			if (remaining < sizeof(pcap_file_header_t))
			{
				fprintf(stderr, "Invalid PCAP file header\n");
				break;
			}

			// pcap_file_header_t *file_header = (pcap_file_header_t *)(data + ctx.current_offset);
			ctx.current_offset += sizeof(pcap_file_header_t);
			remaining -= sizeof(pcap_file_header_t);
		}
		while (remaining >= sizeof(pcap_packet_header_t))
		{
			// 读取PCAP包头部
			pcap_packet_header_t *pcap_hdr = (pcap_packet_header_t *)(data + ctx.current_offset);
			uint32_t packet_len = pcap_hdr->incl_len;

			// 移动到包数据
			ctx.current_offset += sizeof(pcap_packet_header_t);
			remaining -= sizeof(pcap_packet_header_t);

			// 检查是否有足够数据
			if (packet_len > remaining)
			{
				// 包不完整，回退并等待下一个内存块
				if (ctx.mmap_size == MEMORY_MAP_SIZE)
					ctx.current_offset -= sizeof(pcap_packet_header_t);
				else
					ctx.current_offset = ctx.mmap_size;
				break;
			}

			// 解析以太网帧
			const uint8_t *packet_data = data + ctx.current_offset;

			// 跳过以太网头部 (14字节)
			if (packet_len < 14)
			{
				ctx.current_offset += packet_len;
				remaining -= packet_len;
				continue;
			}

			// 检查以太网类型 (0x0800 = IPv4)
			uint16_t eth_type = ntohs(*(uint16_t *)(packet_data + 12));
			if (eth_type != 0x0800)
			{
				// 非IPv4包，跳过
				ctx.current_offset += packet_len;
				remaining -= packet_len;
				continue;
			}

			// 解析IP头部
			const uint8_t *ip_header = packet_data + 14;
			size_t ip_header_len = (ip_header[0] & 0x0F) * 4; // IHL字段

			if (packet_len < 14 + ip_header_len)
			{
				ctx.current_offset += packet_len;
				remaining -= packet_len;
				continue;
			}

			// 检查协议类型 (17 = UDP)
			if (ip_header[9] != 17)
			{
				// 非UDP包，跳过
				ctx.current_offset += packet_len;
				remaining -= packet_len;
				continue;
			}

			// 解析UDP头部
			const uint8_t *udp_header = ip_header + ip_header_len;
			uint16_t udp_len = ntohs(*(uint16_t *)(udp_header + 4));

			// 获取UDP负载
			const uint8_t *payload = udp_header + 8;
			size_t payload_len = udp_len - 8;

			// 解析UDP负载
			if (payload_len >= 4)
			{
				m_log_queue.try_enqueue(std::string((char *)payload, payload_len));
				std::this_thread::sleep_for(std::chrono::nanoseconds(100000000 / cfg->frame_package_num));
				if (payload_len == 1316)
					ctx.pointcloud_num++;
				else if (payload_len == 33)
					ctx.imu_num++;
				else if (payload_len == 112)
					ctx.heart_num++;
				else
					printf("payload_len:%lu\n", payload_len);
			}
			// 移动到下一个包
			ctx.current_offset += packet_len;
			remaining -= packet_len;
			ctx.bytes_processed += packet_len;
		}
		// 更新已处理大小
		ctx.processed_size = ctx.last_mapped_offset + ctx.current_offset;
		// printf("%s %d\n",__FUNCTION__,__LINE__);
		//  printf("ctx.processed_size:%lu %lu %lu %lu\n",ctx.processed_size,ctx.pointcloud_num,ctx.imu_num,ctx.heart_num);
	}
	// 等待统计线程结束

	err = "parse log thread end:pointcloud:" + std::to_string(ctx.pointcloud_num) + "imu:" + std::to_string(ctx.imu_num) + "heart:" + std::to_string(ctx.heart_num);
	WriteLogData(cfg->ID, MSG_DEBUG, (char *)err.c_str(), err.size());
	ctx.running = 0;
	// 清理资源
	destroy_context(&ctx);
}
void PaceCatLidarSDK::PlaybackThreadProc(int id)
{
	RunConfig *cfg = GetConfig(id);
	if (cfg == nullptr)
		return;

	uint64_t frame_starttime = 0;
	int continuous_times = 0;
	double last_ang = 100;
	std::vector<LidarCloudPointData> tmp_filter;
	std::vector<double> tmp_ang;

	cfg->sfp.sfp_enable = false;
	cfg->mr_2.mr_enable = false;
	DeBugInfo debuginfo;
	memset(&debuginfo, 0, sizeof(DeBugInfo));
	debuginfo.imu_packet_idx = -1;
	debuginfo.pointcloud_packet_idx = -1;
	debuginfo.timer = LOG_TIMER * 1000000000ULL; // 定时检测时间
	LidarPacketData *pointclouddata = (LidarPacketData *)malloc(sizeof(LidarPacketData) + sizeof(LidarCloudPointData) * cfg->frame_package_num * 128);
	// save one packet imudata
	LidarPacketData *imudata = (LidarPacketData *)malloc(sizeof(LidarPacketData) + sizeof(LidarImuPointData));
	// 主处理循环
	while (cfg->run_state != QUIT)
	{
		std::string chunk;
		bool ret = m_log_queue.try_dequeue(chunk);
		if (!ret)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
			continue;
		}
		unsigned char *recv_data = (unsigned char *)chunk.c_str();
		int recv_len = chunk.size();
		if ((recv_data[0] == 0x0 || recv_data[0] == 0x1) && recv_data[1] == 0x24)
		{
			// 判定是否是数据包
			const BlueSeaLidarEthernetPacket *packet = (BlueSeaLidarEthernetPacket *)recv_data;
			if (debuginfo.pointcloud_timestamp_last)
			{
				int64_t diff = int64_t(packet->timestamp - debuginfo.pointcloud_timestamp_last);
				if (diff > POINTCLOUD_TIMESTAMP_MAX_DIFF)
				{
					std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud packet interval large:" + std::to_string(diff) + "  " + std::to_string(POINTCLOUD_TIMESTAMP_MAX_DIFF);

					WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
				}
				if (diff < 0)
				{
					std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud packet time jump back:" + std::to_string(packet->timestamp) + "  " + std::to_string(debuginfo.pointcloud_timestamp_last);
					WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
				}
			}
			debuginfo.pointcloud_timestamp_last = packet->timestamp;
			int packet_size = sizeof(BlueSeaLidarEthernetPacket) + packet->dot_num * sizeof(BlueSeaLidarSpherPoint);

			if (recv_len != packet_size)
			{
				std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud packet length error: " + std::to_string(recv_len) + "  " + std::to_string(packet_size);
				WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
				break;
			}
			if (packet->udp_cnt != (uint8_t)debuginfo.pointcloud_packet_idx && debuginfo.pointcloud_packet_idx >= 0)
			{
				debuginfo.pointcloud_timestamp_drop_num++;
				std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud packet lost :last " + std::to_string((uint8_t)debuginfo.pointcloud_packet_idx) + " now: " + std::to_string(packet->udp_cnt) + "index:" + std::to_string(debuginfo.pointcloud_timestamp_drop_num);
				WriteLogData(cfg->ID, MSG_WARM, (char *)err.c_str(), err.size());
			}
			// printf("%d %d\n",debuginfo.pointcloud_packet_idx,packet->udp_cnt);
			debuginfo.pointcloud_packet_idx = packet->udp_cnt + 1;
			if (packet->version == 1)
			{
				// 从数据包的tag标签判定下雨系数以及单双回波 0单回波 1双回波
				if (packet->rt_v1.tag & TAG_WITH_RAIN_DETECT)
					cfg->rain = packet->rt_v1.rain;

				if (packet->rt_v1.tag & TAG_DUAL_ECHO_MODE)
					cfg->echo_mode = 1;
				else
					cfg->echo_mode = 0;

				if (packet->rt_v1.tag & TAG_MIRROR_NOT_STABLE)
				{
					debuginfo.mirror_err_timestamp_num++;
					if (packet->timestamp - debuginfo.mirror_err_timestamp_last > debuginfo.timer)
					{

						std::string err = "time: " + SystemAPI::getCurrentTime() + " mirror " + std::to_string(packet->rt_v1.mirror_rpm) + " index:" + std::to_string(debuginfo.mirror_err_timestamp_num);
						WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
						debuginfo.mirror_err_timestamp_num = 0;
						debuginfo.mirror_err_timestamp_last = packet->timestamp;

						uint32_t error_code = 0;
						setbit(error_code, ERR_MIRROR_NO_STABLE);
						PaceCatLidarSDK::getInstance()->WriteAlarmData(cfg->ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
					}
					continue;
				}
				else
				{
					if ((packet->timestamp - debuginfo.mirror_err_timestamp_last > debuginfo.timer) && (debuginfo.mirror_err_timestamp_num > 0))
					{
						std::string err = "time: " + SystemAPI::getCurrentTime() + " mirror " + std::to_string(packet->rt_v1.mirror_rpm) + " index:" + std::to_string(debuginfo.mirror_err_timestamp_num);
						WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
						debuginfo.mirror_err_timestamp_num = 0;
						debuginfo.mirror_err_timestamp_last = packet->timestamp;

						uint32_t error_code = 0;
						setbit(error_code, ERR_MIRROR_NO_STABLE);
						PaceCatLidarSDK::getInstance()->WriteAlarmData(cfg->ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
					}
				}

				if (packet->rt_v1.tag & TAG_MOTOR_NOT_STABLE)
				{
					debuginfo.motor_err_timestamp_num++;
					if (packet->timestamp - debuginfo.motor_err_timestamp_last > debuginfo.timer)
					{
						std::string err = "time: " + SystemAPI::getCurrentTime() + " main motor " + std::to_string(packet->rt_v1.motor_rpm_x10 / 10.0) + " index:" + std::to_string(debuginfo.motor_err_timestamp_num);
						WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());

						debuginfo.motor_err_timestamp_num = 0;
						debuginfo.motor_err_timestamp_last = packet->timestamp;

						uint32_t error_code = 0;
						setbit(error_code, ERR_MOTOR_NO_STABLE);
						PaceCatLidarSDK::getInstance()->WriteAlarmData(cfg->ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
					}
					continue;
				}
				else
				{
					if ((packet->timestamp - debuginfo.motor_err_timestamp_last > debuginfo.timer) && debuginfo.motor_err_timestamp_num > 0)
					{
						std::string err = "time: " + SystemAPI::getCurrentTime() + " main motor " + std::to_string(packet->rt_v1.motor_rpm_x10 / 10.0) + " index:" + std::to_string(debuginfo.motor_err_timestamp_num);
						WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());

						debuginfo.motor_err_timestamp_num = 0;
						debuginfo.motor_err_timestamp_last = packet->timestamp;

						uint32_t error_code = 0;
						setbit(error_code, ERR_MOTOR_NO_STABLE);
						PaceCatLidarSDK::getInstance()->WriteAlarmData(cfg->ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
					}
				}
			}
			// printf("%s %d %d\n",__FUNCTION__,__LINE__,m_package_num);
			int count = cfg->cloud_data.size();
			if (count == 0)
			{
				cfg->frame_firstpoint_timestamp = packet->timestamp;
				frame_starttime = cfg->frame_firstpoint_timestamp;
			}
			AddPacketToList(packet, cfg->cloud_data, cfg->frame_firstpoint_timestamp, last_ang, tmp_filter, tmp_ang, cfg->sfp, cfg->mr_2);
			count = cfg->cloud_data.size();
			cfg->package_num_idx++;
			if (cfg->frame_package_num < 80)
			{
				std::string err = "time: " + SystemAPI::getCurrentTime() + "too few point in one frame " + std::to_string(cfg->frame_package_num);
				WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
			}
			// if (count >= cfg->frame_package_num * 128)
			if (cfg->package_num_idx >= cfg->frame_package_num)
			{
				// LidarPacketData *dat = (LidarPacketData *)malloc(sizeof(LidarPacketData) + sizeof(LidarCloudPointData) * count);
				LidarCloudPointData *dat2 = (LidarCloudPointData *)pointclouddata->data;
				SystemAPI::GetTimeStamp(true);

				int dirt_flag = 0;
				int point_idx = 0;
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

#if DEBUG_TEST
				float tmp_zero_point_num_factor = 1.0 * m_zero_point_num / m_sum_point_num;
				if (tmp_zero_point_num_factor > ZERO_POINT_FACTOR)
				{
					debuginfo.zero_pointdata_timestamp_num++;
					if (packet->timestamp - debuginfo.zero_pointdata_timestamp_last > debuginfo.timer)
					{
						std::string err = "time: " + SystemAPI::getCurrentTime() + "too many zero point num, scale factor over " + std::to_string(ZERO_POINT_FACTOR) + "value:" + std::to_string(tmp_zero_point_num_factor) + "index:" + std::to_string(debuginfo.zero_pointdata_timestamp_num);
						WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
						debuginfo.zero_pointdata_timestamp_num = 0;
						debuginfo.zero_pointdata_timestamp_last = packet->timestamp;
					}
				}
				else
				{
					if ((packet->timestamp - debuginfo.zero_pointdata_timestamp_last > debuginfo.timer) && (debuginfo.zero_pointdata_timestamp_num > 0))
					{
						std::string err = "time: " + SystemAPI::getCurrentTime() + "too many zero point num, scale factor over " + std::to_string(ZERO_POINT_FACTOR) + "value:" + std::to_string(tmp_zero_point_num_factor) + "index:" + std::to_string(debuginfo.zero_pointdata_timestamp_num);
						WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
						debuginfo.zero_pointdata_timestamp_num = 0;
						debuginfo.zero_pointdata_timestamp_last = packet->timestamp;
					}
				}

				float tmp_distance_close_num_factor = 1.0 * m_distance_close_num / m_sum_point_num;
				if (tmp_distance_close_num_factor > DISTANCE_CLOSE_FACTOR)
				{
					debuginfo.distance_close_timestamp_num++;
					if (packet->timestamp - debuginfo.distance_close_timestamp_last > debuginfo.timer)
					{
						std::string err = "time: " + SystemAPI::getCurrentTime() + "too many  distance close num,scale factor over " + std::to_string(DISTANCE_CLOSE_FACTOR) + "value:" + std::to_string(tmp_distance_close_num_factor) + "index:" + std::to_string(debuginfo.distance_close_timestamp_num);
						WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
						debuginfo.distance_close_timestamp_num = 0;
						debuginfo.distance_close_timestamp_last = packet->timestamp;
					}
				}
				else
				{
					if ((packet->timestamp - debuginfo.distance_close_timestamp_last > debuginfo.timer) && (debuginfo.distance_close_timestamp_num > 0))
					{
						std::string err = "time: " + SystemAPI::getCurrentTime() + "too many  distance close num,scale factor over " + std::to_string(DISTANCE_CLOSE_FACTOR) + "value:" + std::to_string(tmp_distance_close_num_factor) + "index:" + std::to_string(debuginfo.distance_close_timestamp_num);
						WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
						debuginfo.distance_close_timestamp_num = 0;
						debuginfo.distance_close_timestamp_last = packet->timestamp;
					}
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
				cfg->package_num_idx = 0;
			}
		}
		else if (recv_data[0] == 0xfa && recv_data[1] == 0x88)
		{
			TransBuf *trans = (TransBuf *)recv_data;
			IIM42652_FIFO_PACKET_16_ST *imu_stmp = (IIM42652_FIFO_PACKET_16_ST *)(trans->data + 1);
			if (trans->idx != (uint16_t)debuginfo.imu_packet_idx && debuginfo.imu_packet_idx >= 0)
			{
				std::string err = "time: " + SystemAPI::getCurrentTime() + " imudata packet lost :last " + std::to_string((uint16_t)debuginfo.imu_packet_idx) + " now: " + std::to_string(trans->idx) + " index:" + std::to_string(debuginfo.imu_timestamp_drop_num);
				WriteLogData(cfg->ID, MSG_WARM, (char *)err.c_str(), err.size());
			}
			debuginfo.imu_packet_idx = trans->idx + 1;
			if (debuginfo.imu_timestamp_last)
			{
				int64_t diff = int64_t(imu_stmp->timestamp - debuginfo.imu_timestamp_last);
				// printf("2:%d\n",diff);
				if (fabs(diff) > IMU_TIMESTAMP_MAX_DIFF)
				{
					std::string err = "time: " + SystemAPI::getCurrentTime() + " imu packet interval large:" + std::to_string(diff) + "  " + std::to_string(IMU_TIMESTAMP_MAX_DIFF);
					WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
				}
				if (diff < 0)
				{
					std::string err = "time: " + SystemAPI::getCurrentTime() + " imu packet time jump back:" + std::to_string(imu_stmp->timestamp) + "  " + std::to_string(debuginfo.imu_timestamp_last);
					WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
				}
			}
			debuginfo.imu_timestamp_last = imu_stmp->timestamp;
			LidarImuPointData *dat2 = (LidarImuPointData *)imudata->data;

			dat2->gyro_x = imu_stmp->Gyro_X * 4000.0 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[0];
			dat2->gyro_y = imu_stmp->Gyro_Y * 4000.0 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[1];
			dat2->gyro_z = imu_stmp->Gyro_Z * 4000.0 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[2];
			dat2->acc_x = (imu_stmp->Accel_X * 4.0 / 0x10000);
			dat2->acc_y = (imu_stmp->Accel_Y * 4.0 / 0x10000);
			dat2->acc_z = (imu_stmp->Accel_Z * 4.0 / 0x10000);
			// dat2->acc_x = (imu_stmp.Accel_X * 4.0 / 0x10000) * cfg->imu_drift.K[0] + cfg->imu_drift.B[0];
			// dat2->acc_y = (imu_stmp.Accel_Y * 4.0 / 0x10000)* cfg->imu_drift.K[1] + cfg->imu_drift.B[1];
			// dat2->acc_z = (imu_stmp.Accel_Z * 4.0 / 0x10000)* cfg->imu_drift.K[2] + cfg->imu_drift.B[2];
			dat2->linear_acceleration_x = dat2->acc_x * cfg->imu_drift.R[0][0] + dat2->acc_y * cfg->imu_drift.R[0][1] + dat2->acc_z * cfg->imu_drift.R[0][2];
			dat2->linear_acceleration_y = dat2->acc_x * cfg->imu_drift.R[1][0] + dat2->acc_y * cfg->imu_drift.R[1][1] + dat2->acc_z * cfg->imu_drift.R[1][2];
			dat2->linear_acceleration_z = dat2->acc_x * cfg->imu_drift.R[2][0] + dat2->acc_y * cfg->imu_drift.R[2][1] + dat2->acc_z * cfg->imu_drift.R[2][2];
			if (cfg->timemode == 1)
				imudata->timestamp = getCurrentNanoseconds();
			else
				imudata->timestamp = imu_stmp->timestamp;

			imudata->length = sizeof(LidarPacketData) + sizeof(LidarImuPointData);
			imudata->dot_num = 1;
			imudata->frame_cnt = cfg->frame_cnt;
			imudata->data_type = LIDARIMUDATA;
			WriteImuData(cfg->ID, 0, imudata);
			memset(imudata, 0, sizeof(sizeof(LidarPacketData) + sizeof(LidarImuPointData)));
		}
		else if (recv_data[0] == 0xfa && recv_data[1] == 0x89) // debug
		{
		}
		else if (recv_data[0] == 0x4c && recv_data[1] == 0x48 && (unsigned char)recv_data[2] == 0xbe && (unsigned char)recv_data[3] == 0xb4) // time sync
		{
		}
		else if (recv_data[0] == 0x4c && recv_data[1] == 0x48 && (unsigned char)recv_data[2] == 0xac && (unsigned char)recv_data[3] == 0xb8) // dev_param
		{
		}
		else if (recv_data[0] == 0x4c && recv_data[1] == 0x69 && (unsigned char)recv_data[2] == 0x44 && (unsigned char)recv_data[3] == 0x41) // heart
		{
		}
		else if (recv_data[0] == 0x4c && recv_data[1] == 0x48 && (unsigned char)recv_data[2] == 0xbc && (unsigned char)recv_data[3] == 0xff)
		{

			CmdHeader *cmdheader = (CmdHeader *)recv_data;
			if (cmdheader->len == sizeof(SYS_EVENT_LOG))
			{
				std::string syslog = bin_to_hex_fast(recv_data + sizeof(CmdHeader), cmdheader->len);
				WriteLogData(cfg->ID, MSG_WARM, (char *)syslog.c_str(), syslog.size());
			}
			else
			{
				printf("cmd:%d %s\n", cmdheader->len, recv_data + sizeof(CmdHeader));
			}
		}
		else if (recv_data[0] == 0x4c && recv_data[1] == 0x4d && (unsigned char)recv_data[2] == 0x53 && (unsigned char)recv_data[3] == 0x47) // alarm
		{
			// LidarMsgHdr* hdr = (LidarMsgHdr*)(buf);
		}
		else
		{
			const uint8_t *ptr = (uint8_t *)&recv_data;
			printf("%d : %02x %02x %02x %02x %02x %02x %02x %02x\n",
				   recv_len,
				   ptr[0], ptr[1], ptr[2], ptr[3],
				   ptr[4], ptr[5], ptr[6], ptr[7]);
		}
		if (debuginfo.pointcloud_timestamp_last && debuginfo.imu_timestamp_last)
		{
			int64_t diff = int64_t(debuginfo.pointcloud_timestamp_last - debuginfo.imu_timestamp_last);
			if (fabs(diff) > POINTCLOUD_IMU_TIMESTAMP_MAX_DIFF)
			{
				debuginfo.pointcloud_imu_timestamp_large_num++;
				uint64_t new_timestamp = SystemAPI::GetTimeStamp(true);
				if (new_timestamp - debuginfo.pointcloud_imu_timestamp_large_last > debuginfo.timer / 1000000)
				{
					std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud and imu packet timestamp not sync" + std::to_string(diff) + "  " + std::to_string(POINTCLOUD_IMU_TIMESTAMP_MAX_DIFF) + " index:" + std::to_string(debuginfo.pointcloud_imu_timestamp_large_num);
					WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
					debuginfo.pointcloud_imu_timestamp_large_num = 0;
					debuginfo.pointcloud_imu_timestamp_large_last = new_timestamp;
				}
			}
			else
			{
				uint64_t new_timestamp = SystemAPI::GetTimeStamp(true);
				if ((new_timestamp - debuginfo.pointcloud_imu_timestamp_large_last > debuginfo.timer / 1000000) && (debuginfo.pointcloud_imu_timestamp_large_num > 0))
				{
					std::string err = "time: " + SystemAPI::getCurrentTime() + " pointcloud and imu packet timestamp not sync" + std::to_string(diff) + "  " + std::to_string(POINTCLOUD_IMU_TIMESTAMP_MAX_DIFF) + " index:" + std::to_string(debuginfo.pointcloud_imu_timestamp_large_num);
					WriteLogData(cfg->ID, MSG_ERROR, (char *)err.c_str(), err.size());
					debuginfo.pointcloud_imu_timestamp_large_num = 0;
					debuginfo.pointcloud_imu_timestamp_large_last = new_timestamp;
				}
			}
		}
	}

	std::string err = "playback thread  end";
	WriteLogData(cfg->ID, MSG_DEBUG, (char *)err.c_str(), err.size());
}

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

bool PaceCatLidarSDK::ReadCalib(int ID, std::string lidar_ip, int port)
{
#ifdef _WIN32
	WSADATA wsda; //   Structure   to   store   info
	WSAStartup(MAKEWORD(2, 2), &wsda);
#endif // _WIN32
	int sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	std::string errmsg;

	uint8_t zbuf[16] = {0};
	SendNetPack(sockfd, DRIFT_RD_PACK, 4, zbuf, (char *)lidar_ip.c_str(), port);

	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(sockfd, &readfds);

	struct timeval tv;
	tv.tv_sec = 5;
	tv.tv_usec = 0;
	int retval = select(sockfd + 1, &readfds, NULL, NULL, &tv);
	if (retval < 0)
	{
		close(sockfd);
		return retval;
	}
	if (retval == 0)
	{
		close(sockfd);
		return false;
	}

	char buf[1024] = {0};
	sockaddr_in addr;
	socklen_t sz = sizeof(addr);
	recvfrom(sockfd, (char *)&buf, sizeof(buf), 0,
			 (struct sockaddr *)&addr, &sz);

	// close(sockfd);
	const CmdHeader *hdr = (CmdHeader *)buf;
	if (hdr->sign != PACK_PREAMLE)
		return false;

	uint32_t len4 = ((hdr->len + 3) >> 2) * 4;
	const uint32_t *pcrc = (uint32_t *)((char *)buf + sizeof(CmdHeader) + len4);
	uint32_t chk = stm32crc((uint32_t *)buf, len4 / 4 + 2);

	if (*pcrc != chk)
		return false;

	uint16_t type = ~(hdr->cmd);
	if (type != DRIFT_RD_PACK || hdr->len < sizeof(DriftCalib))
		return false;

	DriftCalib drift;
	memcpy(&drift, buf + sizeof(CmdHeader), sizeof(drift));
	if (drift.code != DRIFT_MAGIC)
		return false;

	int id = QueryIDByIp(lidar_ip);
	RunConfig *cfg = GetConfig(id);
	cfg->imu_drift = drift.drifts.imu;
	// char imu_gyro[256] = {0};
	// sprintf(imu_gyro, "imu : %f, %f, %f; %f, %f, %f; %f, %f, %f \ngyro : %f, %f, %f\n",
	// 		cfg->imu_drift.R[0][0], cfg->imu_drift.R[0][1], cfg->imu_drift.R[0][2],
	// 		cfg->imu_drift.R[1][0], cfg->imu_drift.R[1][1], cfg->imu_drift.R[1][2],
	// 		cfg->imu_drift.R[2][0], cfg->imu_drift.R[2][1], cfg->imu_drift.R[2][2],
	// 		cfg->imu_drift.Gyro[0], cfg->imu_drift.Gyro[1], cfg->imu_drift.Gyro[2]);

	return true;
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
	mreq.imr_interface.s_addr = get_interface_ip(heartinfo.adapter.c_str());
	if (setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&mreq, sizeof(mreq)) < 0)
	{
		heartinfo.value = "add broadcast error";
		heartinfo.code = SystemAPI::getLastError();
		SystemAPI::closefd(sock, true);
		return;
	}
	// struct ifreq ifr;
	// memset(&ifr, 0, sizeof(ifr));
	// strncpy(ifr.ifr_name, heartinfo.adapter.c_str(), IFNAMSIZ);
	// int ret=0;
	// if(ret=setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, &ifr, sizeof(ifr))<0)
	// {
	// 	heartinfo.value = "setsockopt(SO_BINDTODEVICE) failed";
	// 	heartinfo.code = SystemAPI::getLastError();
	// 	SystemAPI::closefd(sock, true);
	// 	return;
	// }

	uint64_t currentTimeStamp = SystemAPI::GetTimeStamp(false);
	uint64_t tto = currentTimeStamp + 1;
	socklen_t sz = sizeof(addr);

	while (heartinfo.isrun)
	{
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(sock, &fds);
		struct timeval to = {0, 100};
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
						// printf("%d %d %d\n",devheart->timestamp[0] ,devheart->timestamp[1],heartinfo.lidars[i].timestamp);
						heartinfo.lidars[i].motor_rpm = devheart->motor_rpm;
						heartinfo.lidars[i].mirror_rpm = devheart->mirror_rpm;
						heartinfo.lidars[i].temperature = devheart->temperature;
						heartinfo.lidars[i].voltage = devheart->voltage;

						isexist = true;
						heartinfo.lidars[i].flag = true;
						if (!heartinfo.lidars[i].isonline)
						{
							heartinfo.lidars[i].isonline = true;
							// std::string result = sn + " " + ip +" "+std::to_string(heartinfo.lidars[i].timestamp)+"  online";
							std::string result = sn + " " + ip + "  online";
							WriteLogData(id, 0, (char *)result.c_str(), result.size());
						}
					}
				}
				if (!isexist)
				{
					// 这里限制仅一个雷达
					if (heartinfo.lidars.size() == 0)
					{
						ConnectInfo info;
						info.ip = ip;
						info.sn = sn;
						info.port = devheart->port;
						info.timestamp = (devheart->timestamp[1]) * 1000 + devheart->timestamp[0] / 1000000;
						info.temperature = devheart->temperature;
						info.motor_rpm = devheart->motor_rpm;
						info.mirror_rpm = devheart->mirror_rpm;
						info.voltage = devheart->voltage;
						info.isonline = true;
						info.flag = true;
						heartinfo.lidars.push_back(info);
						// std::string result = sn + " " + ip +" "+std::to_string(info.timestamp)+"  online";
						std::string result = sn + " " + ip + "  online";
						WriteLogData(id, 0, (char *)result.c_str(), result.size());
					}
					else
					{
						std::string result = sn + " " + std::to_string(sn.size())+" "+ip + "recv other heart packet";
						WriteLogData(id, 0, (char *)result.c_str(), result.size());
					}
				}

				// 查询该ip所在的ID，发送严重警告事件
				int ID = PaceCatLidarSDK::getInstance()->QueryIDByIp(ip);
				// 判定条件:温度高于85  转速为0  电压:[10,32]

				// printf("%f %f %d %f\n",devheart->temperature/10.0,devheart->motor_rpm/10.0,devheart->mirror_rpm,devheart->voltage/1000.0);
				uint32_t error_code = 0;
				if (devheart->temperature / 10.0 > 85)
					setbit(error_code, ERR_TEMPERATURE_HIGH);
				if (devheart->motor_rpm / 10.0 == 0)
					setbit(error_code, ERR_MOTOR_ZERO);
				if (devheart->mirror_rpm == 0)
					setbit(error_code, ERR_MIRROR_ZERO);
				if (devheart->voltage / 1000.0 <= 10)
					setbit(error_code, ERR_VOLTAGE_LOW);
				if (devheart->voltage / 1000.0 >= 32)
					setbit(error_code, ERR_VOLTAGE_HIGH);
				if (error_code)
					PaceCatLidarSDK::getInstance()->WriteAlarmData(ID, MSG_CRITICAL, (char *)&error_code, sizeof(int));
			}
		}
		// check is outtime
		currentTimeStamp = SystemAPI::GetTimeStamp(false);
		if (currentTimeStamp > tto)
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
					heartinfo.lidars[i].timestamp = 0;
					std::string result = heartinfo.lidars[i].sn + " " + heartinfo.lidars[i].ip + "  offline";
					WriteLogData(id, 0, (char *)result.c_str(), result.size());
				}
				heartinfo.lidars[i].flag = false;
			}
			tto = currentTimeStamp + 1;
		}
	}
	heartinfo.value = "thread heart end,lidar number:" + std::to_string(heartinfo.lidars.size());
	heartinfo.code = 0;
	heartinfo.lidars.clear();
	heartinfo.isrun = false;
	SystemAPI::closefd(sock, true);
}

bool PaceCatLidarSDK::FirmwareUpgrade(int id, std::string lidarip, int lidartport, int listenport, std::string path)
{
	std::string result;
	// 对文件后缀进行校验
	std::size_t idx = path.find('.');
	if (idx == std::string::npos)
	{
		result = "upgrade file format is error";
		WriteLogData(id, MSG_ERROR, (char *)result.c_str(), result.size());
		return false;
	}
	else
	{
		std::string suffix = path.substr(idx + 1);
		if (suffix != "lhr" && suffix != "lhl")
		{
			result = "upgrade file format is error:" + suffix;
			WriteLogData(id, MSG_ERROR, (char *)result.c_str(), result.size());
			return false;
		}
	}

	FirmwareInfo fileinfo;
	FirmwareInfo Lidarinfo;

	FirmwareFile *firmwareFile = LoadFirmware(path.c_str(), fileinfo);
	result = "firmware file model:" + fileinfo.model + "mcu:" + fileinfo.mcu + "motor:" + fileinfo.motor;
	WriteLogData(id, MSG_DEBUG, (char *)result.c_str(), result.size());

	if (!firmwareFile)
	{
		result = "load bin-file failed ";
		WriteLogData(id, MSG_ERROR, (char *)result.c_str(), result.size());
		return false;
	}
	// 判定是机头固件还是底板固件
	uint8_t firmwaretype;
	if (fileinfo.model == "LDS-M300-HDR")
	{
		result = "firmware file upgrade type  is mcu";
		WriteLogData(id, MSG_DEBUG, (char *)result.c_str(), result.size());
		firmwaretype = FirmwareType::MCU;
	}
	else if (fileinfo.model == "LDS-M300-E")
	{
		result = "firmware file upgrade type  is motor";
		WriteLogData(id, MSG_DEBUG, (char *)result.c_str(), result.size());
		firmwaretype = FirmwareType::MOTOR;
	}
	else
	{
		result = "firmware file model is not true:" + fileinfo.model;
		WriteLogData(id, MSG_ERROR, (char *)result.c_str(), result.size());
		free(firmwareFile);
		return false;
	}

	int fdUdp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	int recv_len = 0;
	char recv_buf[512] = {0};
	int try_time = TRY_TIME;
	while (try_time--)
	{
		CommunicationAPI::udp_talk_pack(fdUdp, lidarip.c_str(), lidartport, 6, "LXVERH", 0x0043, recv_len, recv_buf);
		if (recv_len)
		{
			if (getLidarVersion(recv_buf, recv_len, Lidarinfo))
			{
				result = "lidar file model:" + Lidarinfo.model + "mcu:" + Lidarinfo.mcu + "motor:" + Lidarinfo.motor;
				WriteLogData(id, MSG_DEBUG, (char *)result.c_str(), result.size());
				break;
			}
		}
	}
	if (firmwaretype == FirmwareType::MCU)
	{
		if (Lidarinfo.mcu.find(fileinfo.mcu) != std::string::npos)
		{
			result = "lidar mcu version is  same ,need not upgrade!";
			WriteLogData(id, MSG_ERROR, (char *)result.c_str(), result.size());
			free(firmwareFile);
			close(fdUdp);
			return true;
		}
		// 获取固件的单双回波信息
		int firmmware_echo_mode = -1; // 0为单回波  1为双回波
		if (fileinfo.mcu.find("V211") != std::string::npos)
			firmmware_echo_mode = MCUType::DUAL;
		else if (fileinfo.mcu.find("V101") != std::string::npos)
			firmmware_echo_mode = MCUType::SINGLE;
		else
		{
			result = "firmware file echo mode is not find:" + fileinfo.mcu;
			WriteLogData(id, MSG_ERROR, (char *)result.c_str(), result.size());
			free(firmwareFile);
			close(fdUdp);
			return false;
		}
		std::string firmmware_echo_mode_str = (firmmware_echo_mode == MCUType::SINGLE ? "single echo" : "dual echo");
		result = "firmware file echo mode:" + firmmware_echo_mode_str;
		WriteLogData(id, MSG_DEBUG, (char *)result.c_str(), result.size());

		// 获取雷达的单双回波信息  0为单回波  1为双回波
		int lidar_echo_mode = -2;
		if (Lidarinfo.mcu.find("V211") != std::string::npos)
			lidar_echo_mode = MCUType::DUAL;
		else if (Lidarinfo.mcu.find("V101") != std::string::npos)
			lidar_echo_mode = MCUType::SINGLE;
		else
		{
			result = "lidar info echo mode is not find";
			WriteLogData(id, MSG_ERROR, (char *)result.c_str(), result.size());
			free(firmwareFile);
			close(fdUdp);
			return false;
		}
		std::string lidar_echo_mode_str = (lidar_echo_mode == MCUType::SINGLE ? "single echo" : "dual echo");
		result = "lidar info echo mode:" + lidar_echo_mode_str;
		WriteLogData(id, MSG_DEBUG, (char *)result.c_str(), result.size());

		if (lidar_echo_mode == firmmware_echo_mode)
		{
			firmwareFile->sent = 0;
			RangeUpInfo ru;
			memset(&ru, 0, sizeof(ru));

			strcpy(ru.m_ip, lidarip.c_str());
			ru.m_port = lidartport;
			ru.m_sock = fdUdp;
			result = "start upgrade";
			WriteLogData(id, MSG_DEBUG, (char *)result.c_str(), result.size());
			int ret = UpgradeMCU(&ru, firmwareFile);
			printf("ret:%d\n", ret);
			close(ru.m_sock);
			free(firmwareFile);
			return (ret == 0) ? true : false;
		}
		else
		{
			result = "lidar info echo mode is not same with firmware file";
			WriteLogData(id, MSG_ERROR, (char *)result.c_str(), result.size());
			free(firmwareFile);
			close(fdUdp);
			return false;
		}
	}
	else if (firmwaretype == FirmwareType::MOTOR)
	{
		if (fileinfo.model != Lidarinfo.model)
		{
			result = "lidar model is not same ,can not upgrade!";
			WriteLogData(id, MSG_ERROR, (char *)result.c_str(), result.size());
			free(firmwareFile);
			close(fdUdp);
			return false;
		}
		if (fileinfo.motor == Lidarinfo.motor)
		{
			result = "lidar motor version is  same ,need not upgrade!";
			WriteLogData(id, MSG_ERROR, (char *)result.c_str(), result.size());
			free(firmwareFile);
			close(fdUdp);
			return true;
		}
		int ret = UpgradeMotor(fdUdp, lidarip.c_str(), lidartport, firmwareFile->len, (char *)firmwareFile->buffer);
		free(firmwareFile);
		close(fdUdp);
		return (ret == 0) ? true : false;
	}

	return false;
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

in_addr_t get_interface_ip(const char *ifname)
{
	if (!ifname || strlen(ifname) == 0)
	{
		return INADDR_NONE;
	}

#ifdef _WIN32
	return 0;
#else
	// Linux/Unix 实现 (保持不变)
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd < 0)
	{
		perror("socket");
		return INADDR_NONE;
	}

	struct ifreq ifr;
	strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';

	if (ioctl(fd, SIOCGIFADDR, &ifr) < 0)
	{
		perror("ioctl(SIOCGIFADDR)");
		close(fd);
		return INADDR_NONE;
	}

	close(fd);
	return ((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr.s_addr;
#endif
}
std::string bin_to_hex_fast(const uint8_t *data, size_t length, bool uppercase)
{
	// 创建十六进制字符查找表
	static const char hex_chars_upper[] = "0123456789ABCDEF";
	static const char hex_chars_lower[] = "0123456789abcdef";

	const char *hex_chars = uppercase ? hex_chars_upper : hex_chars_lower;

	// 预分配内存（每个字节对应2个十六进制字符）
	std::string result;
	result.reserve(length * 2);

	for (size_t i = 0; i < length; ++i)
	{
		uint8_t byte = data[i];
		result += hex_chars[(byte >> 4) & 0x0F]; // 高4位
		result += hex_chars[byte & 0x0F];		 // 低4位
	}

	return result;
}
