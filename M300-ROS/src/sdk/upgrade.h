#pragma once
#define TRY_TIME 5
#define TRY_RECV_COUNT 500

#include <stdarg.h>
#include <sys/ipc.h>
#include <sys/stat.h>
#include <sys/msg.h>
#include <termios.h>
#include<stdlib.h>
#include<string.h>
#include<iostream>
#include<stdio.h>
#include <time.h>
#include <netdb.h>

#include"global.h"
#pragma pack (push,1)


struct FirmwareFile
{
	int code;
	int len;
	int sent;
	uint32_t crc;
	uint8_t date[4];
	uint8_t unused[120];
	char describe[512];
	uint8_t buffer[0];
};

struct FirmwarePart
{
	uint32_t offset;
	uint32_t crc;
	uint32_t buf[128];
};

struct FirmWriteResp
{
	uint32_t offset;
	int result;
	char msg[128];
};
struct ResendPack
{
	time_t timeout;
	uint32_t tried;
	uint16_t cmd;
	uint16_t sn;
	uint16_t len;
	char buf[2048];
};

struct RangeUpInfo
{
    int m_aOps[4];
    bool m_bAbort;
    int m_sock;
    char m_ip[32];
    int m_port;
    uint32_t m_resp[256];

    int m_SN;

};
struct CmdBody
{
    unsigned short sign;
    unsigned short cmd;
    unsigned short sn;
    unsigned short len;
    union {
        char txt[4];
        uint8_t buf[4];
    };
};

#pragma pack (pop)

struct FirmwareInfo
{
    std::string model;
    std::string mcu;
    std::string motor;

};

enum FirmwareType
{
    MOTOR=1,
    MCU
};
enum MCUType
{
    SINGLE=1,
    DUAL
};

FirmwareFile *LoadFirmware(const char *path, FirmwareInfo &info);
bool getLidarVersion(char *buf,int len,FirmwareInfo& info);

bool RangerTalk(RangeUpInfo *This, int len, const void *buf, int timeout, bool bHex);
bool RangerTalk(RangeUpInfo *This, const char *cmd, const char *want, int timeout=5000);
bool SearchPattern(const CmdBody *cmd, const char *want);
int SendRanger(int sock, const char *ip, int port, int len, const void *buf, int sn);
bool WaitRangerMsg(int sock, int mxl, CmdBody *msg, int timeout);
void AddLog(RangeUpInfo *This, bool bTx, int sn, const char *txt, int len);
int UpgradeMotor(int fdUdp, const char *devIp, int devPort, int binLen, char *binBuf);
int UpgradeMCU(RangeUpInfo *This, FirmwareFile *m_firmware);

void SendUpgradePack(unsigned int udp, const FirmwarePart* fp, char* ip, int port, int SN, ResendPack *resndBuf);
