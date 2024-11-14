#ifndef __DATA_H__
#define __DATA_H__

#include <stdint.h>
#include<stdio.h>
#include <string.h>
#define UNUSED(x) (void)x

#ifdef _WIN32
#define NOMINMAX
#include <direct.h>		//for mkdir rmdir
#include <io.h>			//for access
#include<ws2tcpip.h>
#include<winsock.h>
#include<Windows.h>
#include <iphlpapi.h>
#pragma comment(lib, "iphlpapi.lib")
#pragma comment(lib, "Advapi32.lib")
#pragma comment(lib,"ws2_32.lib")
#elif __unix__
#include <sys/time.h>
#include <unistd.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>
#include <stdarg.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#endif

#ifdef _WIN32
#define ACCESS _access
#define MKDIR(a) _mkdir((a))
#define RMDIR(a) _rmdir((a))
#elif __linux__
#define ACCESS access
#define MKDIR(a) mkdir((a),0755)
#define RMDIR(a) rmdir((a))
#endif


#define   HEARTPORT  6789

#endif