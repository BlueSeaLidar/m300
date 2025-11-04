#include "upgrade.h"

FirmwareFile *LoadFirmware(const char *path, FirmwareInfo &info)
{
    // ###DEV TYPE:
    // ###MOTOR VERSION:
    // ###MCU VERSION:
    // LDS-M300-E_MOTOR_V401-250108-A_20250109-151321
    FILE *pfile = nullptr;
    unsigned long length = 0; // 升级文件长度
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

    char *file_data = (char *)malloc(length);
    if (file_data == NULL)
    {
        perror("内存分配失败");
        fclose(pfile);
        return NULL;
    }
    if (fread(file_data, 1, length, pfile) != (size_t)length)
    {
        perror("读取文件失败");
        free(file_data);
        fclose(pfile);
        return NULL;
    }

    for (unsigned int i = 0; i < length - 32; i++)
    {
        if (file_data[i] == '#')
        {
            if (strncmp(&file_data[i], "###DEV TYPE:", 12) == 0)
            {
                info.model = BaseAPI::stringfilter(&file_data[i] + 12, 32);
            }
            else if (strncmp(&file_data[i], "###MOTOR VERSION:", 17) == 0)
            {
                info.motor = BaseAPI::stringfilter(&file_data[i + 17], 32);
            }
            else if (strncmp(&file_data[i], "###MCU:", 7) == 0)
            {
                info.mcu = BaseAPI::stringfilter(&file_data[i + 7], 32);
            }
        }
    }
    // old version
    if (info.model.empty())
    {

        std::string pathstr = path;
        std::size_t idx_model = pathstr.find("_");
        std::size_t idx_motor = pathstr.find("MOTOR");
        std::size_t idx_mcu = pathstr.find("MCU");
        // std::size_t idx_end = pathstr.find('.');
        std::size_t idx_other = pathstr.find_last_of('_');
        bool islegal = false;
        if (idx_model != std::string::npos)
        {
            info.model = pathstr.substr(0, idx_model);
        }
        if (idx_motor != std::string::npos)
        {
            info.motor = pathstr.substr(idx_motor + 6, idx_other - idx_motor - 6);
            for (uint16_t j = 0; j < info.motor.size(); j++)
            {
                if (info.motor.c_str()[j] == '-')
                    info.motor[j] = 0x20;
            }
            islegal = true;
        }
        if (idx_mcu != std::string::npos)
        {
            info.mcu = pathstr.substr(idx_mcu + 4, idx_other - idx_mcu - 4);
            for (uint16_t j = 0; j < info.mcu.size(); j++)
            {
                if (info.mcu.c_str()[j] == '-')
                    info.mcu[j] = 0x20;
            }
            islegal = true;
        }
        if (!islegal)
        {
            printf("upgrade filename is not ok\n");
            free(file_data);
            fclose(pfile);
            return NULL;
        }
    }

    fseek(pfile, 0, SEEK_SET);
    FirmwareFile *fp = NULL;
    fp = (FirmwareFile *)file_data;
    fread(fp, length, 1, pfile);
    fclose(pfile);
    if ((unsigned int)fp->code == 0xb18e03ea && fp->len % 512 == 0 && fp->len + sizeof(FirmwareFile) == length)
    {
        if (fp->crc == stm32crc((uint32_t *)(fp->buffer), fp->len / 4))
        {
            return fp;
        }
    }
    free(file_data);
    return NULL;
}
bool getLidarVersion(char *buf, int len, FirmwareInfo &info)
{
    int startidx = 0;
    int endidx = 0;
    int devel = 0;
    for (int i = 0; i < len; i++)
    {
        if ((len >= i + 4) && strncmp(&buf[i], "MCU:", 4) == 0)
        {
            info.model = std::string(&buf[0], i - 1);
            startidx = i + 4;
            devel = 1;
        }
        else if ((len >= i + 14) && strncmp(&buf[i], "MOTOR VERSION:", 14) == 0)
        {
            startidx = i + 14;
            devel = 2;
        }
        else if ((len >= i + 2) && buf[i] == 0x0d && buf[i + 1] == 0x0a)
        {
            endidx = i;
            if (devel == 1)
            {
                info.mcu = std::string(&buf[startidx], endidx - startidx);
            }
            else if (devel == 2)
            {
                info.motor = std::string(&buf[startidx], endidx - startidx);
            }
        }
    }
    if (devel == 2)
        return true;
    else
        return false;
}

bool SearchPattern(const CmdBody *cmd, const char *want)
{
    int n = strlen(want);

    for (int i = 0; i <= cmd->len - n; i++)
    {
        if (memcmp(cmd->txt + i, want, n) == 0)
            return true;
    }
    return false;
}
int SendRanger(int sock, const char *ip, int port, int len, const void *buf, int sn)
{
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(ip);
    addr.sin_port = htons(port);

    char buffer[1024];
    CmdHeader *hdr = (CmdHeader *)buffer;
    hdr->sign = PACK_PREAMLE;
    hdr->cmd = RG_PACK;
    hdr->sn = sn == -1 ? rand() : sn;

    hdr->len = ((len + 3) >> 2) * 4;

    memcpy(buffer + sizeof(CmdHeader), buf, hdr->len);

    // int n = sizeof(CmdHeader);
    unsigned int *pcrc = (unsigned int *)(buffer + sizeof(CmdHeader) + hdr->len);
    pcrc[0] = stm32crc((unsigned int *)(buffer + 0), hdr->len / 4 + 2);

    return sendto(sock, buffer, hdr->len + 12, 0,
                  (struct sockaddr *)&addr, sizeof(struct sockaddr));
}

void AddLog(RangeUpInfo *This, bool bTx, int sn, int len, const uint8_t *buf)
{
    return;
    char line[1024];
    int nl = sprintf(line,
                     "%s[%d] %d: ", bTx ? "TX" : "       RX", sn, len);
    for (int i = 0; i < len; i++)
    {
        nl += sprintf(line + nl,
                      "%02x ", buf[i]);
    }

    printf("%s\n", line);
}
// int g_test=0;
//  退出逻辑:如果没有接收到数据，则指定时间内退出程序，如果超过接收包的数量，则也退出
bool WaitRangerMsg(int sock, int mxl, CmdBody *msg, int timeout)
{
    int try_recv_count = TRY_RECV_COUNT;
    while (try_recv_count--)
    {
        timeval tv;
        tv.tv_sec = timeout / 1000;
        tv.tv_usec = (timeout % 1000) * 1000;

        fd_set fdr;
        FD_ZERO(&fdr);
        FD_SET(sock, &fdr);
        int rt = select(sock + 1, &fdr, 0, 0, &tv);
        if (rt <= 0)
            break;
        if (FD_ISSET(sock, &fdr))
        {
            sockaddr_in addr;
            socklen_t sz = sizeof(addr);

            char *buf = (char *)msg;
            int len = recvfrom(sock, buf, mxl, 0, (sockaddr *)(&addr), &sz);
            if (len > 12 &&
                msg->len + 12 == len &&
                msg->sign == PACK_PREAMLE &&
                msg->cmd == (uint16_t)~RG_PACK)
            {
                unsigned int *crc = (unsigned int *)(buf + sizeof(CmdHeader) + msg->len);
                if (*crc == stm32crc((unsigned int *)buf, msg->len / 4 + 2))
                {
                    msg->txt[msg->len] = 0;
                    return true;
                }
            }
        }
        // printf("g_test：%d\n",g_test++);
    }

    return false;
}
void AddLog(RangeUpInfo *This, bool bTx, int sn, const char *txt, int len)
{
    return;
    char line[1024];
    int nl = sprintf(line, "%s[%d] %d: %s |", bTx ? "TX" : "       RX", sn, len, txt);

    const uint8_t *buf = (const uint8_t *)txt;
    for (int i = 0; i < len; i++)
    {
        nl += sprintf(line + nl, "%02x ", buf[i]);
    }

    printf("%s\n", line);
}

bool RangerTalk(RangeUpInfo *This, int len, const void *buf, int timeout, bool bHex)
{
    CmdBody *cmd = (CmdBody *)(This->m_resp);

    int try_time = TRY_TIME;
    while (try_time--)
    {
        int sn = This->m_SN++ & 0xFFFF;

        SendRanger(This->m_sock, This->m_ip, This->m_port, len, buf, sn);
        if (bHex)
            AddLog(This, true, sn, len, (uint8_t *)buf);
        else
            AddLog(This, true, sn, (char *)buf, len);

        if (!WaitRangerMsg(This->m_sock, sizeof(This->m_resp), cmd, timeout))
            continue;
        AddLog(This, false, cmd->sn, cmd->txt, len);
        if (cmd->sn == sn)
            return true;
    }
    return false;
}

bool RangerTalk(RangeUpInfo *This, const char *cmd, const char *want, int timeout)
{
    if (RangerTalk(This, strlen(cmd), cmd, timeout, false))
    {
        if (want)
        {
            return SearchPattern((CmdBody *)(This->m_resp), want);
        }
    }
    return false;
}

// 封包
int packUdp(int len, void *payload, void *buf)
{
    struct CmdHeader *hdr = (struct CmdHeader *)buf;
    hdr->sign = PACK_PREAMLE;
    hdr->cmd = F_PACK;
    hdr->sn = rand();
    hdr->len = len;

    int len4 = ((len + 3) >> 2) * 4;
    memcpy((char *)buf + UDP_HEADER, (char *)payload, len);

    unsigned int *pcrc = (unsigned int *)((uint8_t *)buf + UDP_HEADER + len4);
    *pcrc = stm32crc((unsigned int *)buf, len4 / 4 + 2);

    return len4 + 12;
}

// 解包
int unpackUdp(int len, uint8_t *buf, struct CmdHeader *hdr, uint8_t **payload)
{
    memcpy(hdr, buf, UDP_HEADER);
    if (hdr->sign != PACK_PREAMLE)
    {
        return -1;
    }

    if (hdr->len + 12 != len)
    {
        return -2;
    }

    unsigned int *crc = (unsigned int *)(buf + UDP_HEADER + hdr->len);
    if (*crc != stm32crc((unsigned int *)buf, hdr->len / 4 + 2))
    {
        return -3;
    }

    *payload = buf + UDP_HEADER;

    return hdr->len;
}

// 命令交互
int udpTalk(int fdUdp, const char *devIp, int devPort,
            int nSend, void *sendBuf,
            int *nRecv, void *recvBuf)
{
    struct sockaddr_in sin1;
    socklen_t sin_size = sizeof(sin1);
    bzero(&sin1, sizeof(struct sockaddr_in));
    sin1.sin_family = AF_INET;
    sin1.sin_addr.s_addr = inet_addr(devIp);
    sin1.sin_port = htons(devPort);

    uint8_t udpBuf[1024];
    int len = packUdp(nSend, sendBuf, udpBuf);
    struct CmdHeader *cmd = (struct CmdHeader *)udpBuf;

    // 发送
    sendto(fdUdp, (char *)udpBuf, len, 0, (struct sockaddr *)&sin1, sin_size);
    int i;
    for (i = 0; i < 3; i++)
    {
        struct timeval to;
        to.tv_sec = 10;
        to.tv_usec = 0;

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fdUdp, &readfds);

        int rt = select(fdUdp + 1, &readfds, NULL, NULL, &to);
        if (rt <= 0)
            continue;
        if (FD_ISSET(fdUdp, &readfds))
        {
            struct sockaddr_in remote;
            socklen_t sin_size = sizeof(remote);

            uint8_t respBuf[1024];
            int len = recvfrom(fdUdp, (char *)respBuf, sizeof(respBuf), 0,
                               (struct sockaddr *)&remote, &sin_size);

            struct CmdHeader resp;
            uint8_t *payload;
            rt = unpackUdp(len, respBuf, &resp, &payload);
            // printf("unpack len %d : %d payload %d\n", len, rt, resp.len);

            if (rt > 0 && resp.sn == cmd->sn)
            {
                *nRecv = resp.len;
                memcpy(recvBuf, payload, resp.len);
                return 0;
            }
            // printf("unknown pack %x len %d\n", resp.cmd, resp.len);
        }
    }
    return -1;
}
int udpSend(int fdUdp, const char *devIp, int devPort,
            int nSend, void *sendBuf)
{
    struct sockaddr_in sin1;
    socklen_t sin_size = sizeof(sin1);
    bzero(&sin1, sizeof(struct sockaddr_in));
    sin1.sin_family = AF_INET;
    sin1.sin_addr.s_addr = inet_addr(devIp);
    sin1.sin_port = htons(devPort);

    uint8_t udpBuf[1024];
    int len = packUdp(nSend, sendBuf, udpBuf);
    // struct CmdHeader *cmd = (struct CmdHeader *)udpBuf;

    // 发送
    sendto(fdUdp, (char *)udpBuf, len, 0, (struct sockaddr *)&sin1, sin_size);
    return 0;
}

int upgradeTalk(int fdUdp, const char *devIp, int devPort,
                struct FirmwarePart *part,
                struct FirmWriteResp *resp)
{
    uint8_t respBuf[1024];
    int nResp;

    // 发送开始升级
    int rt = udpTalk(fdUdp, devIp, devPort, sizeof(struct FirmwarePart), part, &nResp, respBuf);
    if (rt != 0 || (unsigned long)nResp < sizeof(struct FirmWriteResp))
    {
        printf("udp talk error %d %d\n", rt, nResp);
        return -1;
    }
    memcpy(resp, respBuf, sizeof(struct FirmWriteResp));

    // 检查结果
    if (resp->offset != part->offset)
    {
        printf("offset %x != %x\n", resp->offset, part->offset);
        return -2;
    }
    return 0;
}

int UpgradeMotor(int fdUdp, const char *devIp, int devPort, int binLen, char *binBuf)
{
    // 开始升级
    struct FirmwarePart start;
    start.offset = OP_FLASH_ERASE; // 擦除FLASH
    start.crc = 0xffffffff;        //
    start.buf[0] = binLen;         // 长度

    struct FirmWriteResp resp;
    printf("send erase length %d\n", binLen);
    int rt = upgradeTalk(fdUdp, devIp, devPort, &start, &resp);
    if (rt != 0)
    {
        printf("send start, resp %d\n", rt);
        return -1;
    }
    printf("start return %d, resp %d : %s\n", rt, resp.result, resp.msg);

    // 循环发送数据文件
    uint32_t binOffset = 0;
    
    while (binOffset < (uint32_t)binLen)
    {
        // 升级文件片段
        struct FirmwarePart part;
        part.offset = binOffset;
        memcpy(part.buf, binBuf + binOffset, 512); // 每次发送512字节
        part.crc = stm32crc(part.buf, 128);
        int try_time = TRY_TIME;
        while (try_time--)
        {
            // 发送文件片段
            printf("sending part offset %x\n", binOffset);
            rt = upgradeTalk(fdUdp, devIp, devPort, &part, &resp);
            if (rt != 0)
            {
                printf("ng:send part offset %x return %d\n", binOffset, rt);
                continue;
            }
            printf("ok:offset %x return %d : %s\n", binOffset, resp.result, resp.msg);
            if (resp.result == 0)
                break;
        }
        if(try_time<=0)
            return -2;
        binOffset += 512;
    }

    // 传输完成
    struct FirmwarePart iap;
    iap.offset = OP_WRITE_IAP; // 传输完成
    iap.crc = 0xffffffff;
    iap.buf[0] = binLen; // 文件长度

    printf("sending iap length %d\n", binLen);
    rt = upgradeTalk(fdUdp, devIp, devPort, &iap, &resp);
    if (rt != 0)
    {
        printf("send iap error %d\n", rt);
        return -3;
    }
    printf("iap return %d %s\n", resp.result, resp.msg);

    FirmwarePart reset;
    reset.offset = OP_FRIMWARE_RESET; // 重启
    reset.crc = 0xffffffff;
    reset.buf[0] = 0xabcd1234; // 确认

    printf("sending reset\n");
    udpSend(fdUdp, devIp, devPort, sizeof(FirmwarePart), &reset);
    return 0;
}
int UpgradeMCU(RangeUpInfo *This, FirmwareFile *m_firmware)
{
    if (!RangerTalk(This, "LCONNH", "OK"))
        return -1;

    char cmd[32], want[32];
    sprintf(cmd, "LERMM:%dH", m_firmware->len);
    sprintf(want, "Erase %x", m_firmware->len);
    if (!RangerTalk(This, cmd, want))
        return -2;

    printf("The firmware of the machine head has been successfully erased\n");
    // send upgrade file
    while (m_firmware->sent < m_firmware->len)
    {
        char cmd[32], want[32];
        sprintf(cmd, "LNXPG:%xH", m_firmware->sent);
        sprintf(want, "Page %x", m_firmware->sent);

        if (!RangerTalk(This, cmd, want))
            return -3;
        // send mem line
        for (int lino = 0; !This->m_bAbort && lino < PAGE_SIZE / LINE_SIZE; lino++)
        {
            uint8_t line[LINE_SIZE + 8]; // 20];
            line[0] = 0xbb;
            line[1] = lino;
            int ofset = m_firmware->sent + lino * LINE_SIZE;
            memcpy(line + 2, m_firmware->buffer + ofset, LINE_SIZE);

            line[LINE_SIZE + 2] = line[1];
            for (int j = 0; j < LINE_SIZE; j++)
                line[LINE_SIZE + 2] ^= line[2 + j];

            if (RangerTalk(This, LINE_SIZE + 3, line, 300, true))
            {
                if (!SearchPattern((CmdBody *)(This->m_resp), "Line:OK"))
                {
                    return -4;
                }
            }
            else
            {
                return -5;
            }
        }
        uint32_t *pb = (uint32_t *)m_firmware->buffer;
        uint32_t cc = stm32crc(pb + m_firmware->sent / 4, PAGE_SIZE / 4);
        sprintf(cmd, "LWRPG:%xH", cc);

        if (RangerTalk(This, cmd, "MEM:OK"))
        {
            m_firmware->sent += PAGE_SIZE;
        }
        else
        {
            return -6;
        }
    }
    printf("Data transmission successful\n");
    memset(cmd,0,32);
    sprintf(cmd, "LFNMM:%xH", m_firmware->crc);

    if (!RangerTalk(This, cmd, "ALL:OK"))
    {
        return -7;
    }
    printf("upgrade ok!\n");
    sleep(2);
    CommunicationAPI::send_cmd_udp(This->m_sock, This->m_ip, This->m_port, 0x0043, rand(), 6, "LRESTH");
    return 0;
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