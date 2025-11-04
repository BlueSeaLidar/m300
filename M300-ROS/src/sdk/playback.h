#ifndef __PLAYBACK_H__
#define __PLAYBACK_H__
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <arpa/inet.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <time.h>
#include <pthread.h>
#include <signal.h>

// ======================
// PCAP文件格式定义
// ======================
typedef struct pcap_file_header {
    uint32_t magic_number;   // 0xa1b2c3d4
    uint16_t version_major;   // 2
    uint16_t version_minor;   // 4
    int32_t  thiszone;        // GMT to local correction
    uint32_t sigfigs;         // accuracy of timestamps
    uint32_t snaplen;         // max length of captured packets
    uint32_t linktype;        // data link type (1 = Ethernet)
} pcap_file_header_t;

typedef struct pcap_packet_header {
    uint32_t ts_sec;         // timestamp seconds
    uint32_t ts_usec;        // timestamp microseconds
    uint32_t incl_len;       // number of octets of packet saved in file
    uint32_t orig_len;       // actual length of packet
} pcap_packet_header_t;

// ======================
// 配置参数
// ======================
#define MAX_PACKET_SIZE   1500    // 最大UDP包大小
#define MEMORY_MAP_SIZE   (1024 * 1024 * 1024) // 256MB内存映射块
#define STATS_INTERVAL    5       // 统计信息打印间隔(秒)
#define MAX_PROTOCOLS     10      // 支持的最大协议类型

// ======================
// 雷达协议类型定义
// ======================
typedef enum {
    LIDAR_PROTO_UNKNOWN = 0,
    LIDAR_PROTO_TYPE_M300
} LidarProtocolType;

// ======================
// 协议处理函数指针
// ======================
typedef void (*ProtocolParser)(const uint8_t* data, size_t length, 
                               const pcap_packet_header_t* pcap_hdr, 
                               void* context);

// ======================
// 协议处理器结构
// ======================
typedef struct {
    LidarProtocolType type;
    const char* name;
    ProtocolParser parser;
} ProtocolHandler;

// ======================
// 解析上下文
// ======================
typedef struct {
     int fd;                 // 文件描述符
    size_t file_size;       // 文件总大小
    size_t processed_size;  // 已处理大小
    size_t current_offset;  // 当前偏移
    void* mmap_ptr;         // 内存映射指针
    size_t mmap_size;       // 当前映射大小
    size_t last_mapped_offset; // 上次映射的起始偏移
    
    // 统计信息
    uint64_t pointcloud_num;
    uint64_t imu_num;
    uint64_t heart_num;
    uint64_t bytes_processed;
    time_t last_stat_time;
    
    // 协议处理器
    ProtocolHandler handlers[MAX_PROTOCOLS];
    int handler_count;
    
    // 实时打印控制
    volatile sig_atomic_t running;
} ParseContext;

// ======================
// 函数声明
// ======================
int init_context(ParseContext* ctx, const char* filename);
void destroy_context(ParseContext* ctx);
int map_next_chunk(ParseContext* ctx);
int parse_chunk(ParseContext* ctx);
// void register_protocol_handler(ParseContext* ctx, LidarProtocolType type, 
//                               const char* name, ProtocolParser parser);
long get_page_size();

#endif