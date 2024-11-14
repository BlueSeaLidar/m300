﻿// M300_SDK.cpp: 定义应用程序的入口点。
//
#include "sdk.h"
#include"global.h"
#include <fstream>
#ifdef _WIN32
#pragma warning(disable:4996)
#pragma warning(disable:4244)
#endif
#define M_PI  3.14159265358979323846

BlueSeaLidarSDK *BlueSeaLidarSDK::m_sdk = new (std::nothrow) BlueSeaLidarSDK();
BlueSeaLidarSDK * BlueSeaLidarSDK::getInstance()
{
	return m_sdk;
}

void BlueSeaLidarSDK::deleteInstance()
{
	if (m_sdk)
	{
		delete m_sdk;
		m_sdk = NULL;
	}
}
BlueSeaLidarSDK::BlueSeaLidarSDK()
{
}

BlueSeaLidarSDK::~BlueSeaLidarSDK()
{
}
bool BlueSeaLidarSDK::SetPointCloudCallback(int ID, LidarCloudPointCallback cb, void* client_data)
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
    lidar->cloudpoint =client_data;
	return true;
}
bool BlueSeaLidarSDK::SetImuDataCallback(int ID, LidarImuDataCallback cb, void* client_data)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID ==  ID && m_lidars.at(i)->run_state != QUIT)
		{
			lidar = m_lidars.at(i);
			break;
		}
	}
	if (lidar == NULL)
		return false;

	lidar->cb_imudata = cb;
	lidar->imudata = (char*)client_data;
	return true;
}
bool BlueSeaLidarSDK::SetLogDataCallback(int ID, LidarLogDataCallback cb, void* client_data)
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
	lidar->logdata = (char*)client_data;
	return true;
}

void BlueSeaLidarSDK::WritePointCloud(int ID, const uint8_t dev_type, LidarPacketData * data, void * client_data)
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
		lidar->cb_cloudpoint(ID, dev_type, data, client_data);
}

void BlueSeaLidarSDK::WriteImuData(int ID, const uint8_t dev_type, LidarPacketData * data, void * client_data)
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
		lidar->cb_imudata(ID, dev_type, data, client_data);
}

void BlueSeaLidarSDK::WriteLogData( int ID, const uint8_t dev_type, char * data, int len)
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
		return ;
	if (lidar->cb_logdata != nullptr)
		lidar->cb_logdata(ID, dev_type, data, len);
}

void BlueSeaLidarSDK::Init()
{
	m_heartinfo.code = 0;
	m_heartinfo.isrun = true;
	m_heartthread = std::thread(HeartThreadProc, std::ref(m_heartinfo));
	m_heartthread.detach();
}
void BlueSeaLidarSDK::Uninit()
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		m_lidars.at(i)->run_state = QUIT;
	}
}
int BlueSeaLidarSDK::AddLidar(std::string lidar_ip, int lidar_port, int listen_port)
{
	RunConfig *cfg = new RunConfig;
	cfg->lidar_ip = lidar_ip.c_str();
	cfg->lidar_port = lidar_port;
	cfg->listen_port = listen_port;
	cfg->ID = m_lidars.size();
	cfg->run_state = ONLINE;
	cfg->frame_cnt = 0;
	cfg->cb_cloudpoint = NULL;
	cfg->cb_imudata = NULL;
	cfg->cb_logdata = NULL;

	m_lidars.push_back(cfg);
	return cfg->ID;
}
bool BlueSeaLidarSDK::ConnectLidar(int ID)
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

	lidar->thread_subData=std::thread (UDPThreadProc,ID);
	lidar->thread_subData.detach();

	lidar->thread_pubCloud=std::thread(PubCloudThreadProc,ID);
	lidar->thread_pubCloud.detach();


	lidar->thread_pubImu = std::thread(PubImuThreadProc, ID);
	lidar->thread_pubImu.detach();
	return true;
}
bool BlueSeaLidarSDK::DisconnectLidar(int ID)
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

bool BlueSeaLidarSDK::QueryBaseInfo(int ID,BaseInfo & info)
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
	int index = 50;
	while (lidar->action != FINISH && index > 0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		index--;
	}

	if (lidar->action == FINISH)
	{
		lidar->action = NONE;
		if (lidar->recv_len==sizeof(EEpromV101))
		{
			EEpromV101 *eepromv101 = (EEpromV101*)lidar->recv_buf.c_str();
			info.uuid = BaseAPI::stringfilter((char*)eepromv101->dev_sn, 20);
			info.model = BaseAPI::stringfilter((char*)eepromv101->dev_type, 16);

			char tmp_IPv4[16] = { 0 };
			char tmp_mask[16] = { 0 };
			char tmp_gateway[16] = { 0 };
			char tmp_srv_ip[16] = { 0 };
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
bool BlueSeaLidarSDK::QueryVersion(int ID, VersionInfo &info)
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

	
	lidar->send_buf = "LVERSH";
	lidar->send_len = 6;
	lidar->action = GET_VERSION;
	int index = 50;
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
		info.mcu_ver= tmp2.substr(idx2 + 1, idx - idx2 -1);
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
int BlueSeaLidarSDK::QueryDeviceState(int ID)
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
		return OFFLINE;

	std::string ip = lidar->lidar_ip;
	for (unsigned int i = 0; i < m_heartinfo.lidars.size(); i++)
	{
		std::string tmpip = m_heartinfo.lidars[i].ip;
		if (ip == tmpip)
		{
			return ONLINE;
		}
	}
	return OFFLINE;
}

bool BlueSeaLidarSDK::SetLidarNetWork(int ID,std::string ip, std::string mask, std::string gateway, uint16_t port)
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

	char result[64] = { 0 };
	// 对传入的格式校验
	if (!BaseAPI::checkAndMerge(1, (char*)ip.c_str(), (char*)mask.c_str(), (char*)gateway.c_str(), port, result))
	{
		return false;
	}
	char tmp[128] = { 0 };
	sprintf(tmp, "LSUDP:%sH", result);
	lidar->send_len = strlen(tmp);
	lidar->send_buf=tmp;
	lidar->action = SET_NETWORK;

	int index = 50;
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

bool BlueSeaLidarSDK::SetLidarUploadNetWork(int ID, std::string upload_ip, uint16_t upload_port)
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

	char result[50] = { 0 };
	// 对传入的格式校验
	if (!BaseAPI::checkAndMerge(0, (char*)upload_ip.c_str(), (char*)"", (char*)"", upload_port, result))
	{
		return false;
	}
	char tmp[64] = { 0 };
	sprintf(tmp, "LSDST:%sH", result);
	lidar->send_len = strlen(tmp);
	lidar->send_buf = tmp;
	lidar->action = SET_UPLOAD_NETWORK;

	int index = 50;
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

bool BlueSeaLidarSDK::SetLidarUploadFix(int ID, bool isfix)
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


	char tmp[64] = { 0 };
	sprintf(tmp, "LSTFX:%dH", isfix);
	lidar->send_len = strlen(tmp);
	lidar->send_buf = tmp;
	lidar->action = SET_UPLOAD_FIX;
	int index = 50;
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

bool BlueSeaLidarSDK::SetLidarAction(int ID, int action)
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
	else
		return false;

	lidar->action = action;
	int index = 50;
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

bool BlueSeaLidarSDK::SetLidarUpgrade(int ID, std::string path)
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

	//check path is exist
	
	std::ifstream f(path.c_str());
	if (!f.good())
		return false;

	lidar->send_len = path.size();
	lidar->send_buf = path;
	lidar->action = UPGRADE;

	int index = 150;
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

int BlueSeaLidarSDK::QueryIDByIp(std::string ip)
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

RunConfig * BlueSeaLidarSDK::getConfig(int ID)
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
void BlueSeaLidarSDK::PacketToPoints(BlueSeaLidarSpherPoint bluesea, LidarCloudPointData &point)
{
	point.reflectivity = bluesea.reflectivity;

	int32_t theta = bluesea.theta_hi;
	theta = (theta << 12) | bluesea.theta_lo;

	double ang = (90000 - theta) * M_PI / 180000;

	double depth = bluesea.depth / 1000.0;

	double r = depth * cos(ang);
	point.z = depth * sin(ang);

	ang = bluesea.phi * M_PI / 180000;
	point.x = cos(ang) * r;
	point.y = sin(ang) * r;

	point.line = 0;
	point.tag = 0;
}


void BlueSeaLidarSDK::AddPacketToList(const BlueSeaLidarEthernetPacket* packet, std::vector<LidarCloudPointData> &cloud_data,std::mutex &mutex)
{
	std::vector<LidarCloudPointData>tmp;
	for (int i = 0; i < packet->dot_num; i++)
	{
		LidarCloudPointData point;
		PacketToPoints(packet->points[i], point);
		//uint64_t timestamp = packet->timestamp + i * packet->time_interval * 100 / (packet->dot_num - 1);
		tmp.push_back(point);
	}
	mutex.lock();
	for (unsigned int i = 0; i < tmp.size(); i++)
	{
		cloud_data.push_back(tmp.at(i));
	}
	mutex.unlock();
}

void  UDPThreadProc(int id)
{
	RunConfig *cfg=BlueSeaLidarSDK::getInstance()->getConfig(id);

	int fd = SystemAPI::open_socket_port(cfg->listen_port, false);
    if (fd <= 0)
	{
		std::string err = "listen port open failed";
		BlueSeaLidarSDK::getInstance()->WriteLogData(cfg->ID, MSG_ERROR, (char*)err.c_str(), err.size());
		return;
	}
	uint16_t wait_idx = 0;
	uint8_t bluesea_idx = 0;
	uint16_t nlen = 0;
	uint8_t buffer[TRANS_BLOCK * 2];

	//struct timeval tv;
	//SystemAPI::GetTimeStamp(&tv, false);
	//time_t tto = tv.tv_sec + 1;

	while (cfg->run_state != QUIT)
	{
		if (cfg->run_state == OFFLINE)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}

		uint8_t buf[1024];
		sockaddr_in addr;
		socklen_t sz = sizeof(addr);
		int dw = recvfrom(fd, (char*)&buf, sizeof(buf), 0,(struct sockaddr*)&addr, &sz);
		if (dw > 0&&(strcmp((char *)inet_ntoa(addr.sin_addr), cfg->lidar_ip.c_str())==0))
		{
			//  time syn
			//SystemAPI::GetTimeStamp(&tv, false);
			//if (tv.tv_sec > tto)
			//{
			//	KeepAlive alive;
			//	SystemAPI::GetTimeStamp(&tv, false);
			//	alive.world_clock = (tv.tv_sec % 3600) * 1000 + tv.tv_usec / 1000;
			//	alive.delay = 0;
			//	// acknowlege device
			//	CommunicationAPI::send_cmd_udp(fd, cfg->lidar_ip.c_str(), cfg->lidar_port, 0x4b41, rand(), sizeof(alive), &alive);
			//	tto = tv.tv_sec + 1;
			//}
			int packet_size = sizeof(BlueSeaLidarEthernetPacket);
			if (buf[0] == 0 && dw == packet_size)
			{
				const BlueSeaLidarEthernetPacket* packet = (BlueSeaLidarEthernetPacket*)&buf;

				if (packet->udp_cnt != bluesea_idx)
				{
					std::string err = "packet lost :last " + std::to_string(packet->udp_cnt) + " now: " + std::to_string(packet_size);
					BlueSeaLidarSDK::getInstance()->WriteLogData(cfg->ID, MSG_WARM, (char*)err.c_str(), err.size());
				}
				bluesea_idx = packet->udp_cnt + 1;
				BlueSeaLidarSDK::getInstance()->AddPacketToList(packet, cfg->cloud_data, cfg->data_mutex);
				//printf("%d %X %d %d\n", __LINE__, cfg, cfg->run_state, cfg->cloud_data.size());
			}
			else if (buf[0] == 0xfa && buf[1] == 0x88)
			{
				const TransBuf* trans = (TransBuf*)buf;
				if (trans->idx != wait_idx) {
					std::string err = "packet lost :last " + std::to_string(trans->idx) + " now: " + std::to_string(wait_idx);
					BlueSeaLidarSDK::getInstance()->WriteLogData(cfg->ID, MSG_WARM, (char*)err.c_str(), err.size());
				}

				memcpy(buffer + nlen, trans->data, trans->len);
				nlen += trans->len;
				wait_idx = trans->idx + 1;

				uint16_t n = BlueSeaLidarSDK::getInstance()->Decode(nlen, buffer, cfg->imu_data, cfg->imu_mutex);
				for (int i = 0; i < nlen - n; i++) {
					buffer[i] = buffer[n + i];
				}
				nlen -= n;

			}
			else if (buf[0] == 0xfa && buf[1] == 0x89)
			{

			}
			else if (buf[0] == 0x4c && buf[1] == 0x48 && (unsigned char)buf[2] == 0xbe && (unsigned char)buf[3] ==0xb4)
			{

			}
			else if (buf[0] == 0x4c && buf[1] == 0x48 && (unsigned char)buf[2] ==0xac && (unsigned char)buf[3] == 0xb8)
			{

			}
			else if (buf[0] == 0x4c && buf[1] == 0x4d && (unsigned char)buf[2] == 0x53 && (unsigned char)buf[3] == 0x47)
			{
				//LidarMsgHdr* hdr = (LidarMsgHdr*)(buf);
			}
			else {
				const uint8_t* ptr = (uint8_t*)&buf;
				printf("%d : %02x %02x %02x %02x %02x %02x %02x %02x\n",
					dw,
					ptr[0], ptr[1], ptr[2], ptr[3],
					ptr[4], ptr[5], ptr[6], ptr[7]);
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
			char tmpresult[64] = { 0 };
			if (!CommunicationAPI::udp_talk_pack(fd, cfg->lidar_ip.c_str(), cfg->lidar_port, cfg->send_len, cfg->send_buf.c_str(), C_PACK, cfg->recv_len, tmpresult))
			{
				cfg->recv_buf = "NG";
			}
			cfg->recv_buf = tmpresult;
			cfg->recv_len = cfg->recv_buf.size();
			cfg->action = FINISH;
			break;
		}
		case GET_PARAMS:
		{
			char tmpresult[512] = { 0 };
			if (!CommunicationAPI::udp_talk_pack(fd, cfg->lidar_ip.c_str(), cfg->lidar_port, cfg->send_len, cfg->send_buf.c_str(),GS_PACK, cfg->recv_len,tmpresult))
			{
				cfg->recv_buf = "NG";
				cfg->recv_len = 2;
			}
			cfg->recv_buf = std::string(tmpresult,cfg->recv_len);
			cfg->action = FINISH;
			break;
		}
		case GET_VERSION:
		{
			char tmpresult[128] = { 0 };
			if (!CommunicationAPI::udp_talk_pack(fd, cfg->lidar_ip.c_str(), cfg->lidar_port, cfg->send_len, cfg->send_buf.c_str(), C_PACK,cfg->recv_len, tmpresult))
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
			if (!CommunicationAPI::udp_talk_pack(fd, cfg->lidar_ip.c_str(), cfg->lidar_port, cfg->send_len, cfg->send_buf.c_str(),S_PACK,cfg->recv_len, (char*)cfg->recv_buf.c_str()))
			{
				cfg->recv_buf = "NG";
				cfg->recv_len = 2;
			}
			cfg->action = FINISH;
			break;
		}
		case SET_UPLOAD_FIX:
		{
			char tmpresult[64] = { 0 };
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
			bool ret = BlueSeaLidarSDK::getInstance()->firmwareUpgrade_udp(cfg->lidar_ip.c_str(), cfg->lidar_port, cfg->send_buf, cfg->recv_buf);
			cfg->recv_len = cfg->recv_buf.size();
			if (ret)
			{
				cfg->recv_buf = "OK";
				cfg->recv_len = 2;
			}
			cfg->action = FINISH;
			break;
		}
		
		}
	}
	SystemAPI::closefd(fd,true);


	std::string err = "thread sub end";
	BlueSeaLidarSDK::getInstance()->WriteLogData(cfg->ID, MSG_DEBUG, (char*)err.c_str(), err.size());

}

void PubCloudThreadProc(int id)
{
	RunConfig *cfg = BlueSeaLidarSDK::getInstance()->getConfig(id);
	
	while (cfg->run_state!= QUIT)
	{
		if(cfg->run_state == OFFLINE)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}
		int count = cfg->cloud_data.size();
		if (count > ONE_FRAME_POINT_NUM)
		{
			LidarPacketData *dat = (LidarPacketData*)malloc(sizeof(LidarPacketData) + sizeof(LidarCloudPointData)*count);
			LidarCloudPointData*dat2 = (LidarCloudPointData*)dat->data;
			struct timeval tv;
			SystemAPI::GetTimeStamp(&tv, true);
			for (int i = 0; i < count; i++)
			{
				dat2[i] = cfg->cloud_data[i];
			}

			dat->timestamp = tv.tv_sec * 1000000 + tv.tv_usec;
			dat->length = sizeof(LidarPacketData) + sizeof(LidarCloudPointData)*count;
			dat->dot_num = count;
			dat->frame_cnt = cfg->frame_cnt++;
			dat->data_type = LIDARPOINTCLOUD;

			cfg->data_mutex.lock();
			BlueSeaLidarSDK::getInstance()->WritePointCloud(cfg->ID, 0,dat, nullptr);
			cfg->cloud_data.clear();
			cfg->data_mutex.unlock();
		}
		else
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		
	}

}

void PubImuThreadProc(int id)
{
	RunConfig *cfg = BlueSeaLidarSDK::getInstance()->getConfig(id);
	while (cfg->run_state != QUIT)
	{
		if(cfg->run_state == OFFLINE)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}
		if (!cfg->imu_data.empty()) 
		{

			//printf("imu queue %ld\n", cfg->imu_data.size());
			cfg->imu_mutex.lock();
			IIM42652_FIFO_PACKET_16_ST imu_stmp = cfg->imu_data.front();
			cfg->imu_data.pop();
			cfg->imu_mutex.unlock();
			struct timeval tv;
			SystemAPI::GetTimeStamp(&tv, true);

			LidarPacketData *dat = (LidarPacketData*)malloc(sizeof(LidarPacketData) + sizeof(LidarImuPointData));
			LidarImuPointData*dat2 = (LidarImuPointData*)dat->data;


			dat2->gyro_x = imu_stmp.Gyro_X* 4000.0 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[0];
			dat2->gyro_y = imu_stmp.Gyro_Y* 4000.0 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[1];
			dat2->gyro_z = imu_stmp.Gyro_Z * 4000.0 / 0x10000 * M_PI / 180 + cfg->imu_drift.Gyro[2];
			dat2->acc_x = (imu_stmp.Accel_X * 4.0 / 0x10000);//* cfg->imu_drift.K[0] + cfg->imu_drift.B[0];
			dat2->acc_y = (imu_stmp.Accel_Y * 4.0 / 0x10000);//* cfg->imu_drift.K[1] + cfg->imu_drift.B[1];
			dat2->acc_z = (imu_stmp.Accel_Z * 4.0 / 0x10000);//* cfg->imu_drift.K[2] + cfg->imu_drift.B[2];
			dat2->linear_acceleration_x = dat2->acc_x * cfg->imu_drift.R[0][0] + dat2->acc_y * cfg->imu_drift.R[0][1] + dat2->acc_z * cfg->imu_drift.R[0][2];
			dat2->linear_acceleration_y = dat2->acc_x * cfg->imu_drift.R[1][0] + dat2->acc_y * cfg->imu_drift.R[1][1] + dat2->acc_z * cfg->imu_drift.R[1][2];
			dat2->linear_acceleration_z = dat2->acc_x * cfg->imu_drift.R[2][0] + dat2->acc_y * cfg->imu_drift.R[2][1] + dat2->acc_z * cfg->imu_drift.R[2][2];
			dat->timestamp = imu_stmp.timestamp;
			dat->length = sizeof(LidarPacketData) + sizeof(LidarImuPointData);
			dat->dot_num = 1;
			dat->frame_cnt = cfg->frame_cnt;
			dat->data_type = LIDARIMUDATA;
			BlueSeaLidarSDK::getInstance()->WriteImuData(cfg->ID, 0, dat, nullptr);

			//std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
		else
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		
	}

}

uint16_t BlueSeaLidarSDK::Decode(uint16_t n, const uint8_t* buf, std::queue<IIM42652_FIFO_PACKET_16_ST>&imu_data,std::mutex &imu_mutex)
{
	uint16_t idx = 0;
	int ndrop = 0;
	uint8_t drop[1024];
	while (idx < n)
	{
		uint8_t ch = buf[idx];

		if (ch == 0xdd)
		{
			if (idx + 4 > n) break;
			//AddRawPoint(buf + idx);
			idx += 4;
		}
		else if (ch == 0xdc || ch == 0xde)
		{
			if (idx + 14 > n) break;
			uint16_t v = buf[idx + 4];
			v = (v << 8) + buf[idx + 5];
			uint32_t ts[2];
			memcpy(ts, buf + idx + 6, 8);
			idx += 14;
		}
		else if (ch == 0xd3) {
			if (idx + 25 > n) break;
			 
			IIM42652_FIFO_PACKET_16_ST* imu= (IIM42652_FIFO_PACKET_16_ST*)(buf + idx + 1);
			imu_mutex.lock();
			imu_data.push(*imu);
			imu_mutex.unlock();
			idx += 25;
		}
		else if (ch == 0xee)
		{
			if (idx + 6 > n) break;
			idx += 6;
		}
		else {
			drop[ndrop++] = ch;
			idx++;
		}
	}
	if (ndrop > 0) {
		printf("drop %d : %02x %02x %02x %02x %02x %02x\n",
			ndrop,
			drop[0], drop[1], drop[2],
			drop[3], drop[4], drop[5]);
	}

    return idx;
}


int BlueSeaLidarSDK::PackNetCmd(uint16_t type, uint16_t len, uint16_t sn, const void* buf, uint8_t* netbuf)
{
	CmdHeader* hdr = (CmdHeader*)netbuf;

	hdr->sign = PACK_PREAMLE;
	hdr->cmd = type;// S_PACK;
	hdr->sn = sn;// rand();

	hdr->len = len;
	int len4 = ((len + 3) >> 2) * 4;
	if (len > 0) {
		memcpy(netbuf + sizeof(CmdHeader), buf, len);
	}

	//int n = sizeof(CmdHeader);
	uint32_t* pcrc = (uint32_t*)(netbuf + sizeof(CmdHeader) + len4);
	pcrc[0] = stm32crc((uint32_t*)(netbuf + 0), len4 / 4 + 2);

	return len4 + 12;
}

int BlueSeaLidarSDK::SendNetPack(int sock, uint16_t type, uint16_t len, const void* buf,char*ip,int port)
{
	uint8_t netbuf[1024];
	int netlen = PackNetCmd(type, len, rand(), buf, netbuf);

	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr(ip);
	addr.sin_port = htons(port);

	return sendto(sock, (char*)netbuf, netlen, 0,
		(struct sockaddr*)&addr, sizeof(struct sockaddr));
}


int BlueSeaLidarSDK::read_calib(int sockfd, const char* lidar_ip, int port, IMUDrift  &imu_drift)
{
	printf("read data from %s:%d\n", lidar_ip, port);
	uint8_t zbuf[16] = { 0 };
	SendNetPack(sockfd, DRIFT_RD_PACK, 4, zbuf, (char*)lidar_ip, port);

	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(sockfd, &readfds);

	struct timeval tv;
	tv.tv_sec = 3;
	tv.tv_usec = 0;
	int retval = select(sockfd + 1, &readfds, NULL, NULL, &tv);
	if (retval < 0) {
		printf("sock err\n");
		return retval;
	}
	if (retval == 0) {
		printf("sock timeout\n");
		return -1;
	}

	char buf[1024];
	sockaddr_in addr;
	socklen_t sz = sizeof(addr);
	recvfrom(sockfd, (char*)&buf, sizeof(buf), 0,
		(struct sockaddr*)&addr, &sz);

	const CmdHeader* hdr = (CmdHeader*)buf;
	if (hdr->sign != PACK_PREAMLE)
	{
		printf("unknown response\n");
		return -1;
	}

	uint32_t len4 = ((hdr->len + 3) >> 2) * 4;
	const uint32_t* pcrc = (uint32_t*)((char*)buf + sizeof(CmdHeader) + len4);
	uint32_t chk = stm32crc((uint32_t*)buf, len4 / 4 + 2);

	if (*pcrc != chk) {
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
	if (drift.code != DRIFT_MAGIC) {
		printf("bad drift data\n");
		return -1;
	}

	imu_drift = drift.drifts.imu;

	printf("imu : %f, %f, %f; %f, %f, %f; %f, %f, %f;",
		imu_drift.R[0][0], imu_drift.R[0][1], imu_drift.R[0][2],
		imu_drift.R[1][0], imu_drift.R[1][1], imu_drift.R[1][2],
		imu_drift.R[2][0], imu_drift.R[2][1], imu_drift.R[2][2]);

	printf("gyro : %f, %f, %f\n",
		imu_drift.Gyro[0], imu_drift.Gyro[1], imu_drift.Gyro[2]);

	return 0;
}

void HeartThreadProc(HeartInfo &heartinfo)
{
#ifdef _WIN32
	WSADATA   wsda; //   Structure   to   store   info
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

	int iResult = ::bind(sock, (struct sockaddr*)&addr, sizeof(addr));
	if (iResult != 0)
	{		
		heartinfo.value ="bind port failed";
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
	/*int time_out = 1000;
	if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&time_out, sizeof(time_out)) == -1)
	{
		heartinfo.value = "socket init error";
		heartinfo.code = SystemAPI::getLastError();
		SystemAPI::closefd(sock, true);
		return;
	}*/

	struct timeval tv;
	SystemAPI::GetTimeStamp(&tv, false);
	time_t tto = tv.tv_sec +3;
	while(heartinfo.isrun)
	{
		socklen_t sz = sizeof(addr);

		char raw[4096];
		int dw = recvfrom(sock, raw, sizeof(raw), 0, (struct sockaddr*)&addr, &sz);
		if (dw == sizeof(DevHeart))
		{
			DevHeart *devheart = (DevHeart *)raw;
			std::string sn = BaseAPI::stringfilter(devheart->dev_sn, 20);
			char tmp_ip[16];
			sprintf(tmp_ip, "%d.%d.%d.%d", devheart->ip[0], devheart->ip[1], devheart->ip[2], devheart->ip[3]);
			std::string ip = BaseAPI::stringfilter(tmp_ip, 16);
			int id = BlueSeaLidarSDK::getInstance()->QueryIDByIp(ip);
			bool isexist = false;
			for (unsigned int i = 0; i < heartinfo.lidars.size(); i++)
			{
				if (sn == heartinfo.lidars[i].sn)
				{
					heartinfo.lidars[i].timestamp= devheart->timestamp[1] * 1000 + devheart->timestamp[0] / 1000000;
					isexist = true;
					heartinfo.lidars[i].flag = true;
					if (!heartinfo.lidars[i].state)
					{
						heartinfo.lidars[i].state = true;
						
						std::string result = sn + " " + ip + "  online";
						BlueSeaLidarSDK::getInstance()->WriteLogData(id,0, (char*)result.c_str(), result.size());
					}
					break;

				}
				//exist  two more lidars with same ip   
				if (ip == heartinfo.lidars[i].ip)
				{
					heartinfo.value = "multiple lidars with the same ip";
					heartinfo.code = -1;
					//heartinfo.isrun=false;
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
				info.state = true;
				info.flag = true;
				heartinfo.lidars.push_back(info);
				std::string result = sn + " " + ip + "  online";
				BlueSeaLidarSDK::getInstance()->WriteLogData(id, 0,  (char*)result.c_str(), result.size());
				
				
			}
		}
		//check is outtime
		SystemAPI::GetTimeStamp(&tv, false);
		if (tv.tv_sec > tto)
		{
			for (unsigned int i = 0; i < heartinfo.lidars.size(); i++)
			{
				if (heartinfo.lidars[i].state == ONLINE && !heartinfo.lidars[i].flag)
				{
					int id = BlueSeaLidarSDK::getInstance()->QueryIDByIp(heartinfo.lidars[i].ip);
					heartinfo.lidars[i].state = false;
					std::string result = heartinfo.lidars[i].sn + " " + heartinfo.lidars[i].ip + "  offline";
					BlueSeaLidarSDK::getInstance()->WriteLogData(id, 0, (char*)result.c_str(), result.size());
				}
				heartinfo.lidars[i].flag = false;
			}
			tto = tv.tv_sec +3;

		}
	}
	SystemAPI::closefd(sock, true);
}

bool BlueSeaLidarSDK::firmwareUpgrade_udp(std::string  ip, int port, std::string path, std::string &error)
{
	int id = QueryIDByIp(ip);
	if (id < 0)
		return false;
	std::string result = "firmware update start";
	BlueSeaLidarSDK::getInstance()->WriteLogData(id, 0, (char*)result.c_str(), result.size());

	int udp = SystemAPI::open_socket_port(port, true);
	if (udp <= 0)
	{
		result = "open listen port failed: " + std::to_string(port);
		BlueSeaLidarSDK::getInstance()->WriteLogData(id, 0, (char*)result.c_str(), result.size());
		return false;
	}
	unsigned short rands = rand();
	ResendPack resndBuf;
	FirmwareFile *firmwareFile = LoadFirmware(path.c_str());
	FirmwarePart fp;
	if (!firmwareFile)
	{
		result = "Failed to load firmware upgrade file: " + std::to_string(port);
		BlueSeaLidarSDK::getInstance()->WriteLogData(id, 0, (char*)result.c_str(), result.size());
		SystemAPI::closefd(udp, true);
		return false;
	}
	result = "load firmware file ok";
	BlueSeaLidarSDK::getInstance()->WriteLogData(id, 0, (char*)result.c_str(), result.size());

	FirmWriteResp* resp = NULL;
	bool isBreak = false;
	fp.offset = OP_FLASH_ERASE;
	fp.buf[0] = firmwareFile->len;
	fp.buf[1] = firmwareFile->crc;
	memset(&resndBuf, 0, sizeof(ResendPack));
	int idx = 0;
	SendUpgradePack(udp, &fp, (char*)ip.c_str(), port, rands, &resndBuf);
	result = "send flash erase";
	BlueSeaLidarSDK::getInstance()->WriteLogData(id, 0, (char*)result.c_str(), result.size());
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
		struct timeval to = { 5, 0 };
		int ret = select(udp + 1, &fds, NULL, NULL, &to);
		if (ret < 0)
		{
			continue;
		}
		else if (ret == 0)
			continue;

		if (resndBuf.timeout < time(NULL))
		{
			SendUpgradePack(udp, &fp, (char*)ip.c_str(), port, rands, &resndBuf);
			idx++;
			if (idx > 3)
			{
				//int offset=((FirmwarePart*)(resndBuf.buf))->offset;

				result = "send cmd more than three times without answering";
				BlueSeaLidarSDK::getInstance()->WriteLogData(id, 0, (char*)result.c_str(), result.size());
				idx = 0;
				break;
			}
		}
		if (FD_ISSET(udp, &fds))
		{
			sockaddr_in addr;
			socklen_t sz = sizeof(addr);

			char buf[1024] = { 0 };
			int nr = recvfrom(udp, buf, sizeof(buf), 0, (struct sockaddr*)&addr, &sz);
			if (nr > 0)
			{
				CmdHeader* hdr = (CmdHeader*)buf;
				if (hdr->sign != 0x484c)
					continue;

				if (hdr->sn == rands)
				{
					if (resndBuf.sn == hdr->sn)
					{
						memset(&resndBuf, 0, sizeof(ResendPack));
					}
					resp = new FirmWriteResp;
					memcpy((void*)resp, buf + sizeof(CmdHeader), hdr->len);
					resp->msg[sizeof(resp->msg) - 1] = 0;
					if (resp->result != 0)
						break;
					idx = 0;
					if (resp->offset == OP_WRITE_IAP)
					{
						result = "receive OP_WRITE_IAP";
						BlueSeaLidarSDK::getInstance()->WriteLogData(id, 0, (char*)result.c_str(), result.size());
						if (resp->result == 0)
						{
							FirmwarePart fp;
							//重启机器
							fp.offset = OP_FRIMWARE_RESET;
							fp.buf[0] = 0xabcd1234;
							SendUpgradePack(udp, &fp, (char*)ip.c_str(), port, rands, &resndBuf);
							SystemAPI::closefd(udp, true);
							result = "firmware file update success";
							BlueSeaLidarSDK::getInstance()->WriteLogData(id, 0, (char*)result.c_str(), result.size());
							return true;
						}
						else
						{
							error = resp->result;
							SystemAPI::closefd(udp, true);
							result = "firmware file update failed:"+error;
							BlueSeaLidarSDK::getInstance()->WriteLogData(id, 0, (char*)result.c_str(), result.size());

							return false;
						}
					}
					if (firmwareFile != NULL)
					{
						if (resp->offset == OP_FLASH_ERASE)
						{
							fp.offset = 0;
							result = "receive OP_FLASH_ERASE" ;
							BlueSeaLidarSDK::getInstance()->WriteLogData(id, 0, (char*)result.c_str(), result.size());

						}
						else
						{
							//printf("%x\n",fp.offset);
							fp.offset = resp->offset + 512;
						}

						if (fp.offset + 512 <= (uint32_t)firmwareFile->len)
						{
							result = "offset:"+ std::to_string(fp.offset)+"total:"+ std::to_string(firmwareFile->len);
							BlueSeaLidarSDK::getInstance()->WriteLogData(id, 0, (char*)result.c_str(), result.size());

							memcpy(fp.buf, firmwareFile->buffer + fp.offset, 512);
							fp.crc = stm32crc(fp.buf, 128);
						}
						else
						{
							//MYLOG<<"OP_WRITE_IAP";
							fp.offset = OP_WRITE_IAP;
							fp.buf[0] = firmwareFile->len;
							fp.buf[1] = firmwareFile->crc;
							result = "send OP_WRITE_IAP";
							BlueSeaLidarSDK::getInstance()->WriteLogData(id, 0, (char*)result.c_str(), result.size());
						}
						SendUpgradePack(udp, &fp, (char*)ip.c_str(), port, rands, &resndBuf);
					}
				}
			}
		}
	}
	SystemAPI::closefd(udp, true);
	return false;
}
FirmwareFile* LoadFirmware(const char* path)
{
	FILE* pfile = nullptr;
	long length = 0;//升级文件长度
	pfile = fopen(path, "rb");
	if (pfile == NULL) {
		printf("open error\n");
		return NULL;
	}
	else
	{
		fseek(pfile, 0, SEEK_END);
		length = ftell(pfile);
		fseek(pfile, 0, SEEK_SET);
	}
	FirmwareFile* fp = NULL;
	fp = (FirmwareFile*) new char[length];
	fread(fp, length, 1, pfile);
	fclose(pfile);
	//printf("%d %d  %d  %d\n",__LINE__,fp->len,sizeof(FirmwareFile),length);

	if ((unsigned int)fp->code == 0xb18e03ea && fp->len % 512 == 0 && fp->len + sizeof(FirmwareFile) == (unsigned long)length)
	{
		if (fp->crc == stm32crc((uint32_t*)(fp->buffer), fp->len / 4))
		{

			return fp;
		}
	}
	return NULL;
}


void SendUpgradePack(unsigned int udp, const FirmwarePart* fp, char* ip, int port, int SN, ResendPack *resndBuf)
{
	CommunicationAPI::send_cmd_udp(udp, ip, port, F_PACK, SN, sizeof(FirmwarePart), (char*)fp);
	if (resndBuf) {
		resndBuf->cmd = F_PACK;
		resndBuf->len = sizeof(FirmwarePart);
		resndBuf->tried = 1;
		resndBuf->sn = SN;
		resndBuf->timeout = time(NULL) + TIMEOUT;
		memcpy(resndBuf->buf, fp, sizeof(FirmwarePart));
	}
}