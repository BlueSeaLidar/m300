
#include"playback.h"
#include<errno.h>

// ======================
// 初始化解析上下文
// ======================
int init_context(ParseContext* ctx, const char* filename) {
    // 打开文件
    ctx->fd = open(filename, O_RDONLY);
    if (ctx->fd == -1) {
        perror("open failed");
        return -1;
    }

    // 获取文件大小
    struct stat st;
    if (fstat(ctx->fd, &st) == -1) {
        perror("fstat failed");
        close(ctx->fd);
        return -1;
    }
    ctx->file_size = st.st_size;
    ctx->processed_size = 0;
    ctx->current_offset = 0;
    ctx->mmap_ptr = NULL;
    ctx->mmap_size = 0;
    
    // 初始化统计
    ctx->pointcloud_num = 0;
    ctx->imu_num = 0;
    ctx->bytes_processed = 0;
    ctx->last_stat_time = time(NULL);
    
    // 初始化协议处理器
    ctx->handler_count = 0;
    
    return 0;
}

// ======================
// 清理资源
// ======================
void destroy_context(ParseContext* ctx) {
    if (ctx->mmap_ptr) {
        munmap(ctx->mmap_ptr, ctx->mmap_size);
    }
    if (ctx->fd != -1) {
        close(ctx->fd);
    }
}
// 获取系统页大小
long get_page_size() {
     long page_size = 0;
    if (page_size == 0) {
        page_size = sysconf(_SC_PAGESIZE);
        if (page_size == -1) {
            perror("sysconf failed");
            return -1;
        }
    }
    return page_size;
}
// ======================
// 映射下一个文件块
// ======================
int map_next_chunk(ParseContext* ctx) {
    long page_size = get_page_size();
    if (page_size <= 0) {
        return -1;
    }
    
    // 计算本次映射大小
    size_t remaining = ctx->file_size - ctx->processed_size;
    if (remaining == 0) {
        return 0; // 文件已处理完
    }
    
    // 确保偏移量对齐
    size_t aligned_offset = (ctx->processed_size / page_size) * page_size;
    size_t offset_adjustment = ctx->processed_size - aligned_offset;
    
    // 计算映射大小（包含对齐调整）
    size_t map_size = remaining + offset_adjustment;
    if (map_size > MEMORY_MAP_SIZE) {
        map_size = MEMORY_MAP_SIZE;
    }
    
    // 确保映射大小是页大小的倍数
    map_size = (map_size / page_size) * page_size;
    if (map_size == 0) {
        // 当剩余数据小于页大小时，映射最小大小
        map_size = page_size;
    }
    
    // 检查映射是否超出文件范围
    if (aligned_offset + map_size > ctx->file_size) {
        map_size = ctx->file_size - aligned_offset;
    }
    
    // 取消之前的映射
    if (ctx->mmap_ptr) {
        munmap(ctx->mmap_ptr, ctx->mmap_size);
    }
    
    // 创建新映射
    ctx->mmap_ptr = mmap(NULL, map_size, PROT_READ, MAP_PRIVATE, ctx->fd, aligned_offset);
    if (ctx->mmap_ptr == MAP_FAILED) {
        // 处理小文件映射的特殊情况
        if (map_size < (size_t)page_size) {
            // 尝试直接映射剩余部分
            map_size = remaining;
            ctx->mmap_ptr = mmap(NULL, map_size, PROT_READ, MAP_PRIVATE, ctx->fd, aligned_offset);
            if (ctx->mmap_ptr == MAP_FAILED) {
                fprintf(stderr, "mmap failed for small chunk: %s (offset: %zu, size: %zu)\n", 
                        strerror(errno), aligned_offset, map_size);
                return -1;
            }
        } else {
            fprintf(stderr, "mmap failed: %s (offset: %zu, size: %zu)\n", 
                    strerror(errno), aligned_offset, map_size);
            return -1;
        }
    }
    
    ctx->mmap_size = map_size;
    ctx->last_mapped_offset = aligned_offset;
    ctx->current_offset = offset_adjustment;
    
    return 0;
}

// ======================
// 解析当前内存块
// ======================
int parse_chunk(ParseContext* ctx) 
{
   const uint8_t* data = (const uint8_t*)ctx->mmap_ptr;
    size_t remaining = ctx->mmap_size - ctx->current_offset;
    
    // 如果是文件开头，跳过PCAP全局头
    if (ctx->processed_size == 0) {
        if (remaining < sizeof(pcap_file_header_t)) {
            fprintf(stderr, "Invalid PCAP file header\n");
            return -1;
        }
        
        //pcap_file_header_t* file_header = (pcap_file_header_t*)(data + ctx->current_offset);
        ctx->current_offset += sizeof(pcap_file_header_t);
        remaining -= sizeof(pcap_file_header_t);
    }
    
    while (remaining >= sizeof(pcap_packet_header_t)) {
        // 读取PCAP包头部
        pcap_packet_header_t* pcap_hdr = (pcap_packet_header_t*)(data + ctx->current_offset);
        uint32_t packet_len = pcap_hdr->incl_len;
        
        // 移动到包数据
        ctx->current_offset += sizeof(pcap_packet_header_t);
        remaining -= sizeof(pcap_packet_header_t);
        
        // 检查是否有足够数据
        if (packet_len > remaining) {
            // 包不完整，回退并等待下一个内存块
            ctx->current_offset -= sizeof(pcap_packet_header_t);
            //ctx->current_offset = ctx->mmap_size; 
            break;
        }
        
        // 解析以太网帧
        const uint8_t* packet_data = data + ctx->current_offset;
        
        // 跳过以太网头部 (14字节)
        if (packet_len < 14) {
            ctx->current_offset += packet_len;
            remaining -= packet_len;
            continue;
        }
        
        // 检查以太网类型 (0x0800 = IPv4)
        uint16_t eth_type = ntohs(*(uint16_t*)(packet_data + 12));
        if (eth_type != 0x0800) {
            // 非IPv4包，跳过
            ctx->current_offset += packet_len;
            remaining -= packet_len;
            continue;
        }
        
        // 解析IP头部
        const uint8_t* ip_header = packet_data + 14;
        size_t ip_header_len = (ip_header[0] & 0x0F) * 4; // IHL字段
        
        if (packet_len < 14 + ip_header_len) {
            ctx->current_offset += packet_len;
            remaining -= packet_len;
            continue;
        }
        
        // 检查协议类型 (17 = UDP)
        if (ip_header[9] != 17) {
            // 非UDP包，跳过
            ctx->current_offset += packet_len;
            remaining -= packet_len;
            continue;
        }
        
        // 解析UDP头部
        const uint8_t* udp_header = ip_header + ip_header_len;
        uint16_t udp_len = ntohs(*(uint16_t*)(udp_header + 4));
        
        // 获取UDP负载
        //const uint8_t* payload = udp_header + 8;
        size_t payload_len = udp_len - 8;
        
        // 更新统计
        // 解析UDP负载
        if (payload_len >= 4) {
            if(payload_len==1316)
                ctx->pointcloud_num++;
            if(payload_len==33)
                ctx->imu_num++;

        }
        // 移动到下一个包
        ctx->current_offset += packet_len;
        remaining -= packet_len;
        ctx->bytes_processed += packet_len;
    }
    
    // 更新已处理大小
    ctx->processed_size = ctx->last_mapped_offset + ctx->current_offset;
    
    return 0;
}
