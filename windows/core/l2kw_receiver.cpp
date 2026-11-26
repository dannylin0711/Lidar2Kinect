/*
 * The L2K1 receiver: one UDP socket, one thread. Follows shared/tools/l2k_dump.c: answer every
 * HELLO with a CONFIG for the streams the game has open, keep the newest skeleton frame,
 * reassemble and decode depth (D1 / raw) and colour (JPEG), notice when the sender goes silent.
 */
#include "l2kw_base.h"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdio.h>
#include <string.h>

namespace l2kw {

#define DEPTH_PIXELS (L2K_DEPTH_WIDTH * L2K_DEPTH_HEIGHT)
#define COLOR_BYTES (L2K_COLOR_WIDTH * L2K_COLOR_HEIGHT * 4)

static HMODULE g_module;
static unsigned g_kind = L2K_CONSUMER_FAKE_KINECT;
static INIT_ONCE g_once = INIT_ONCE_STATIC_INIT;

/* receiver thread state */
static SOCKET g_sock = INVALID_SOCKET;
static uint8_t g_depth_store[DEPTH_PIXELS * 3];
static uint8_t g_color_store[L2K_MAX_CHUNKS * L2K_CHUNK_PAYLOAD];
static uint16_t g_depth_px[DEPTH_PIXELS];
static uint8_t g_color_px[COLOR_BYTES];
static l2k_assembler g_depth_as, g_color_as;

struct Stats {
    unsigned hello, skel, depth, color, lost, bad, late, depth_bytes, color_bytes, color_fail;
};

void init_module(HMODULE module, unsigned consumer_kind, const char *product) {
    g_module = module;
    g_kind = consumer_kind;
    if (product) set_product_name(product);
}

static void send_config(const sockaddr_in &to, uint32_t session, uint32_t *seq, uint64_t echo_us) {
    uint8_t pkt[128];
    l2k_config cfg;
    memset(&cfg, 0, sizeof cfg);
    cfg.echo_sender_time_us = echo_us;
    cfg.receiver_time_us = now_us();
    cfg.streams = hub_wanted();
    cfg.skeleton_flags = hub_skeleton_flags();
    cfg.consumer_kind = (uint8_t)config().consumer_kind;
    cfg.depth_mode = (uint8_t)config().depth_mode;
    cfg.color_quality = (uint8_t)config().color_quality;
    strncpy(cfg.name, config().name, L2K_NAME_SIZE);
    size_t len = l2k_encode_config(pkt, sizeof pkt, session, ++*seq, &cfg);
    if (len) sendto(g_sock, (const char *)pkt, (int)len, 0, (const sockaddr *)&to, sizeof to);
}

static DWORD WINAPI receiver_main(void *) {
    const uint64_t timeout_us = (uint64_t)L2K_PEER_TIMEOUT_MS * 1000;
    uint64_t start = now_us();
    uint32_t my_session = (uint32_t)(start ^ (start >> 32) ^ GetCurrentProcessId()) | 1u, my_seq = 0;
    uint32_t peer_session = 0, peer_seq = 0, last_wanted = 0, last_skel_frame = 0;
    bool have_peer = false, present = false, have_skel = false;
    uint64_t last_packet = 0, last_report = start, last_config_sent = 0;
    sockaddr_in peer;
    SenderInfo info;
    Stats st;

    memset(&peer, 0, sizeof peer);
    memset(&info, 0, sizeof info);
    memset(&st, 0, sizeof st);
    l2k_assembler_init(&g_depth_as, g_depth_store, sizeof g_depth_store);
    l2k_assembler_init(&g_color_as, g_color_store, sizeof g_color_store);
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);

    for (;;) {
        static uint8_t pkt[2048];
        sockaddr_in from;
        int from_len = sizeof from;
        memset(&from, 0, sizeof from);
        fd_set rd;
        timeval tv = {0, 2000};
        FD_ZERO(&rd);
        FD_SET(g_sock, &rd);
        int ready = select(0, &rd, NULL, NULL, &tv);
        uint64_t now = now_us();
        int n = ready > 0 ? recvfrom(g_sock, (char *)pkt, sizeof pkt, 0, (sockaddr *)&from, &from_len) : -1;

        if (n > 0) {
            l2k_header h;
            if (l2k_decode_header(pkt, (size_t)n, &h) != L2K_OK) {
                st.bad++;
                continue;
            }
            if (!have_peer || h.session != peer_session) {
                /* a new sender, or the sender restarted */
                log("receiver: sender session %08x from %s:%u%s", h.session, inet_ntoa(from.sin_addr), ntohs(from.sin_port),
                    have_peer ? " (replaces the previous one)" : "");
                have_peer = true;
                peer_session = h.session;
                peer_seq = h.seq - 1;
                have_skel = false;
                l2k_assembler_init(&g_depth_as, g_depth_store, sizeof g_depth_store);
                l2k_assembler_init(&g_color_as, g_color_store, sizeof g_color_store);
            }
            if (l2k_seq_newer(h.seq, peer_seq)) {
                st.lost += h.seq - peer_seq - 1;
                peer_seq = h.seq;
            }
            peer = from;
            last_packet = now;
            switch (h.type) {
            case L2K_PKT_HELLO: {
                l2k_hello hello;
                if (l2k_decode_hello(pkt, (size_t)n, &hello) != L2K_OK) {
                    st.bad++;
                    break;
                }
                st.hello++;
                if (!present || strcmp(info.name, hello.name) || info.caps != hello.caps) {
                    strncpy(info.name, hello.name, L2K_NAME_SIZE);
                    info.caps = hello.caps;
                    info.source_kind = hello.source_kind;
                    log("receiver: HELLO from \"%s\" at %s:%u, caps 0x%x, source %u, max tracked %u", hello.name, inet_ntoa(from.sin_addr),
                        ntohs(from.sin_port), hello.caps, hello.source_kind, hello.max_tracked);
                }
                if (!present) {
                    present = true;
                    info.present = true;
                    hub_sender_present(true, info);
                }
                send_config(peer, my_session, &my_seq, hello.sender_time_us);
                last_config_sent = now;
                last_wanted = hub_wanted();
                break;
            }
            case L2K_PKT_SKELETON: {
                l2k_skeleton_frame f;
                if (l2k_decode_skeleton(pkt, (size_t)n, &f) != L2K_OK) {
                    st.bad++;
                    break;
                }
                if (have_skel && !l2k_seq_newer(f.frame_number, last_skel_frame)) {
                    st.late++;
                    break;
                }
                have_skel = true;
                last_skel_frame = f.frame_number;
                st.skel++;
                if (present) hub_skeleton(f);
                break;
            }
            case L2K_PKT_DEPTH:
            case L2K_PKT_COLOR: {
                l2k_chunk c;
                bool depth = h.type == L2K_PKT_DEPTH;
                l2k_assembler *as = depth ? &g_depth_as : &g_color_as;
                if (l2k_decode_chunk(pkt, (size_t)n, &c) != L2K_OK) {
                    st.bad++;
                    break;
                }
                if (l2k_assembler_push(as, &c) != 1) break;
                if (depth) {
                    bool ok = false;
                    if (as->info.width != L2K_DEPTH_WIDTH || as->info.height != L2K_DEPTH_HEIGHT) {
                        st.bad++;
                        break;
                    }
                    if (as->info.encoding == L2K_DEPTH_D1) ok = l2k_depth_d1_decode(as->data, as->info.total_len, g_depth_px, DEPTH_PIXELS) == 0;
                    else if (as->info.encoding == L2K_DEPTH_RAW && as->info.total_len == DEPTH_PIXELS * 2) {
                        for (int k = 0; k < DEPTH_PIXELS; k++) g_depth_px[k] = (uint16_t)(as->data[k * 2] | (as->data[k * 2 + 1] << 8));
                        ok = true;
                    }
                    if (!ok) {
                        st.bad++;
                        break;
                    }
                    st.depth++;
                    st.depth_bytes += as->info.total_len;
                    if (present) hub_depth(g_depth_px, as->info.frame_number);
                } else {
                    bool ok = false;
                    if (as->info.width != L2K_COLOR_WIDTH || as->info.height != L2K_COLOR_HEIGHT) {
                        st.bad++;
                        break;
                    }
                    if (as->info.encoding == L2K_COLOR_JPEG) ok = jpeg_decode_bgrx(as->data, as->info.total_len, g_color_px, L2K_COLOR_WIDTH, L2K_COLOR_HEIGHT);
                    else if (as->info.encoding == L2K_COLOR_BGRX && as->info.total_len == COLOR_BYTES) {
                        memcpy(g_color_px, as->data, COLOR_BYTES);
                        ok = true;
                    }
                    if (!ok) {
                        st.color_fail++;
                        break;
                    }
                    st.color++;
                    st.color_bytes += as->info.total_len;
                    if (present) hub_color(g_color_px, as->info.frame_number);
                }
                break;
            }
            case L2K_PKT_BYE:
                log("receiver: BYE from \"%s\"", info.name);
                have_peer = false;
                if (present) {
                    present = false;
                    info.present = false;
                    hub_sender_present(false, info);
                }
                break;
            default:
                break;
            }
            continue; /* drain the socket before doing anything else */
        }

        /* nothing waiting */
        if (present && now - last_packet > timeout_us) {
            log("receiver: sender \"%s\" silent for %u ms, treating it as gone", info.name, L2K_PEER_TIMEOUT_MS);
            present = false;
            info.present = false;
            have_peer = false;
            hub_sender_present(false, info);
        }
        if (present) {
            /* the game opened or closed a stream: tell the sender now rather than at the next HELLO */
            uint32_t wanted = hub_wanted();
            if (wanted != last_wanted && now - last_config_sent > 50000) {
                log("receiver: streams wanted 0x%x -> 0x%x", last_wanted, wanted);
                send_config(peer, my_session, &my_seq, 0);
                last_config_sent = now;
                last_wanted = wanted;
            }
        }
        hub_tick();
        if (now - last_report >= 5000000) {
            if (present || st.hello || st.bad) {
                double s = (double)(now - last_report) / 1e6;
                log("receiver: last %.0f s: hello %u, skeleton %.1f/s, depth %.1f/s (%u B), colour %.1f/s (%u B), lost %u, late %u, bad %u, colour decode failures %u",
                    s, st.hello, st.skel / s, st.depth / s, st.depth ? st.depth_bytes / st.depth : 0, st.color / s,
                    st.color ? st.color_bytes / st.color : 0, st.lost, st.late, st.bad, st.color_fail);
            }
            memset(&st, 0, sizeof st);
            last_report = now;
        }
    }
}

static BOOL CALLBACK start_once(PINIT_ONCE, PVOID, PVOID *) {
    WSADATA w;
    sockaddr_in a;
    HMODULE self;
    int big = 1 << 21;

    config_load(g_module, g_kind);
    log_open();
    /* the receiver thread runs code from this module until the process ends: never unload it */
    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN, (LPCWSTR)&start_once, &self);

    if (WSAStartup(MAKEWORD(2, 2), &w) != 0) {
        log("receiver: WSAStartup failed");
        return TRUE;
    }
    g_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (g_sock == INVALID_SOCKET) {
        log("receiver: socket() failed (%d)", WSAGetLastError());
        return TRUE;
    }
    setsockopt(g_sock, SOL_SOCKET, SO_RCVBUF, (const char *)&big, sizeof big);
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons((u_short)config().port);
    if (bind(g_sock, (sockaddr *)&a, sizeof a) != 0) {
        log("receiver: cannot bind UDP port %u (%d) - is another receiver running?", config().port, WSAGetLastError());
        closesocket(g_sock);
        g_sock = INVALID_SOCKET;
        return TRUE;
    }
    {
        /* do not let an ICMP "port unreachable" from a vanished sender break recvfrom */
        BOOL off = FALSE;
        DWORD ret = 0;
        WSAIoctl(g_sock, _WSAIOW(IOC_VENDOR, 12) /* SIO_UDP_CONNRESET */, &off, sizeof off, NULL, 0, &ret, NULL, NULL);
    }
    log("receiver: listening on UDP %u", config().port);
    if (config().dnssd) dnssd_start(config().port, config().name);
    HANDLE t = CreateThread(NULL, 0, receiver_main, NULL, 0, NULL);
    if (t) CloseHandle(t);
    else log("receiver: CreateThread failed (%lu)", GetLastError());
    return TRUE;
}

void start() { InitOnceExecuteOnce(&g_once, start_once, NULL, NULL); }

} // namespace l2kw
