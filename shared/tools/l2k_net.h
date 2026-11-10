/* Minimal UDP + clock shim for the command line tools (Winsock / BSD sockets). Not part of the library. */
#ifndef L2K_NET_H
#define L2K_NET_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
typedef SOCKET net_socket;
typedef int net_socklen;
#define NET_INVALID INVALID_SOCKET
static int net_init(void) {
    WSADATA w;
    return WSAStartup(MAKEWORD(2, 2), &w) == 0;
}
static uint64_t net_now_us(void) {
    LARGE_INTEGER f, c;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    return (uint64_t)(c.QuadPart / f.QuadPart) * 1000000ull + (uint64_t)(c.QuadPart % f.QuadPart) * 1000000ull / (uint64_t)f.QuadPart;
}
static void net_sleep_us(uint64_t us) { Sleep((DWORD)((us + 999) / 1000)); }
static void net_nonblocking(net_socket s) {
    u_long on = 1;
    ioctlsocket(s, FIONBIO, &on);
}
static void net_close(net_socket s) { closesocket(s); }
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
typedef int net_socket;
typedef socklen_t net_socklen;
#define NET_INVALID (-1)
static int net_init(void) { return 1; }
static uint64_t net_now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ull + (uint64_t)ts.tv_nsec / 1000ull;
}
static void net_sleep_us(uint64_t us) { usleep((useconds_t)us); }
static void net_nonblocking(net_socket s) { fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK); }
static void net_close(net_socket s) { close(s); }
#endif

/* UDP socket bound to `port` on all interfaces (port 0 = any), non-blocking. */
static net_socket net_udp(uint16_t port) {
    struct sockaddr_in a;
    int big = 1 << 20;
    net_socket s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s == NET_INVALID) return s;
    setsockopt(s, SOL_SOCKET, SO_RCVBUF, (const char *)&big, sizeof big);
    setsockopt(s, SOL_SOCKET, SO_SNDBUF, (const char *)&big, sizeof big);
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons(port);
    if (bind(s, (struct sockaddr *)&a, sizeof a) != 0) {
        net_close(s);
        return NET_INVALID;
    }
    net_nonblocking(s);
    return s;
}

#endif /* L2K_NET_H */
