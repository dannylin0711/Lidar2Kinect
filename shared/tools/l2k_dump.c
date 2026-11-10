/*
 * l2k_dump - reference L2K1 receiver. Listens for a Lidar2Kinect sender, answers its HELLO with a
 * CONFIG asking for streams, decodes everything it gets and prints one status line per second.
 * It is the smallest complete receiver: the Windows fake Kinect and the game port follow the same steps.
 *
 *   l2k_dump [-p port] [-s streams] [-m depth_mode] [-o dir] [-t seconds]
 *     -s  stream bits wanted: 1 skeleton, 2 depth, 4 colour (default 7)
 *     -m  0 = depth on player pixels only (default), 1 = full depth
 *     -o  write the newest depth image (depth.pgm, 16-bit mm; players.pgm) and colour image (color.jpg) there
 *     -t  stop after this many seconds (default: run until killed)
 */
#include "l2k/l2k.h"
#include "l2k_net.h"

#include <stdlib.h>

#define DEPTH_PIXELS (L2K_DEPTH_WIDTH * L2K_DEPTH_HEIGHT)

static uint8_t depth_store[DEPTH_PIXELS * 3];
static uint8_t color_store[L2K_MAX_CHUNKS * L2K_CHUNK_PAYLOAD];
static uint16_t depth_px[DEPTH_PIXELS];

static void write_file(const char *dir, const char *name, const char *header, const void *data, size_t len) {
    char path[1024];
    FILE *f;
    snprintf(path, sizeof path, "%s/%s", dir, name);
    f = fopen(path, "wb");
    if (!f) return;
    if (header) fputs(header, f);
    fwrite(data, 1, len, f);
    fclose(f);
}

static void save_depth(const char *dir) {
    static uint8_t be[DEPTH_PIXELS * 2], pl[DEPTH_PIXELS];
    char header[64];
    int i;
    for (i = 0; i < DEPTH_PIXELS; i++) {
        uint16_t mm = l2k_depth_mm(depth_px[i]);
        be[i * 2] = (uint8_t)(mm >> 8);
        be[i * 2 + 1] = (uint8_t)mm;
        pl[i] = (uint8_t)(l2k_depth_player(depth_px[i]) * 36);
    }
    snprintf(header, sizeof header, "P5\n%d %d\n65535\n", L2K_DEPTH_WIDTH, L2K_DEPTH_HEIGHT);
    write_file(dir, "depth.pgm", header, be, sizeof be);
    snprintf(header, sizeof header, "P5\n%d %d\n255\n", L2K_DEPTH_WIDTH, L2K_DEPTH_HEIGHT);
    write_file(dir, "players.pgm", header, pl, sizeof pl);
}

int main(int argc, char **argv) {
    uint16_t port = L2K_DEFAULT_PORT;
    uint32_t want = L2K_STREAM_SKELETON | L2K_STREAM_DEPTH | L2K_STREAM_COLOR;
    uint8_t depth_mode = L2K_DEPTH_MODE_PLAYERS;
    const char *outdir = NULL;
    double run_seconds = 0;
    net_socket sock;
    l2k_assembler depth_as, color_as;
    l2k_skeleton_frame skel;
    uint32_t my_session, my_seq = 0, peer_session = 0, peer_seq = 0;
    int have_peer = 0, have_skel = 0, i;
    uint64_t start, last_report, last_packet = 0;
    unsigned n_skel = 0, n_depth = 0, n_color = 0, n_lost = 0, n_bad = 0, depth_bytes = 0, color_bytes = 0, players_px = 0;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-p") && i + 1 < argc) port = (uint16_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "-s") && i + 1 < argc) want = (uint32_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "-m") && i + 1 < argc) depth_mode = (uint8_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "-o") && i + 1 < argc) outdir = argv[++i];
        else if (!strcmp(argv[i], "-t") && i + 1 < argc) run_seconds = atof(argv[++i]);
        else {
            fprintf(stderr, "usage: l2k_dump [-p port] [-s streams] [-m depth_mode] [-o dir] [-t seconds]\n");
            return 2;
        }
    }
    if (!net_init() || (sock = net_udp(port)) == NET_INVALID) {
        fprintf(stderr, "cannot open UDP port %u\n", (unsigned)port);
        return 1;
    }
    l2k_assembler_init(&depth_as, depth_store, sizeof depth_store);
    l2k_assembler_init(&color_as, color_store, sizeof color_store);
    start = last_report = net_now_us();
    my_session = (uint32_t)(start ^ (start >> 32)) | 1u;
    printf("l2k_dump: listening on UDP %u, wanting streams 0x%x\n", (unsigned)port, (unsigned)want);
    fflush(stdout);

    for (;;) {
        static uint8_t pkt[2048];
        struct sockaddr_in from;
        net_socklen from_len = sizeof from;
        uint64_t now = net_now_us();
        int n = (int)recvfrom(sock, (char *)pkt, sizeof pkt, 0, (struct sockaddr *)&from, &from_len);
        if (n > 0) {
            l2k_header h;
            if (l2k_decode_header(pkt, (size_t)n, &h) != L2K_OK) {
                n_bad++;
                continue;
            }
            if (!have_peer || h.session != peer_session) {
                /* a new sender, or the sender restarted: forget the old streams */
                have_peer = 1;
                peer_session = h.session;
                peer_seq = h.seq - 1;
                have_skel = 0;
                l2k_assembler_init(&depth_as, depth_store, sizeof depth_store);
                l2k_assembler_init(&color_as, color_store, sizeof color_store);
            }
            if (l2k_seq_newer(h.seq, peer_seq)) {
                n_lost += h.seq - peer_seq - 1;
                peer_seq = h.seq;
            }
            last_packet = now;
            switch (h.type) {
            case L2K_PKT_HELLO: {
                l2k_hello hello;
                l2k_config cfg;
                size_t len;
                if (l2k_decode_hello(pkt, (size_t)n, &hello) != L2K_OK) { n_bad++; break; }
                memset(&cfg, 0, sizeof cfg);
                cfg.echo_sender_time_us = hello.sender_time_us;
                cfg.receiver_time_us = now;
                cfg.streams = want & hello.caps;
                cfg.consumer_kind = L2K_CONSUMER_MONITOR;
                cfg.depth_mode = depth_mode;
                strcpy(cfg.name, "l2k_dump");
                len = l2k_encode_config(pkt, sizeof pkt, my_session, ++my_seq, &cfg);
                sendto(sock, (const char *)pkt, (int)len, 0, (struct sockaddr *)&from, from_len);
                if (my_seq == 1) {
                    printf("sender \"%s\" at %s:%u  caps 0x%x  source %u  max tracked %u\n", hello.name, inet_ntoa(from.sin_addr),
                           (unsigned)ntohs(from.sin_port), (unsigned)hello.caps, (unsigned)hello.source_kind, (unsigned)hello.max_tracked);
                    fflush(stdout);
                }
                break;
            }
            case L2K_PKT_SKELETON: {
                l2k_skeleton_frame f;
                if (l2k_decode_skeleton(pkt, (size_t)n, &f) != L2K_OK) { n_bad++; break; }
                if (have_skel && !l2k_seq_newer(f.frame_number, skel.frame_number)) break; /* late */
                skel = f;
                have_skel = 1;
                n_skel++;
                break;
            }
            case L2K_PKT_DEPTH:
            case L2K_PKT_COLOR: {
                l2k_chunk c;
                l2k_assembler *as = h.type == L2K_PKT_DEPTH ? &depth_as : &color_as;
                if (l2k_decode_chunk(pkt, (size_t)n, &c) != L2K_OK) { n_bad++; break; }
                if (l2k_assembler_push(as, &c) != 1) break;
                if (h.type == L2K_PKT_DEPTH) {
                    int ok = 0, k;
                    if (as->info.width != L2K_DEPTH_WIDTH || as->info.height != L2K_DEPTH_HEIGHT) { n_bad++; break; }
                    if (as->info.encoding == L2K_DEPTH_D1) ok = l2k_depth_d1_decode(as->data, as->info.total_len, depth_px, DEPTH_PIXELS) == 0;
                    else if (as->info.encoding == L2K_DEPTH_RAW && as->info.total_len == DEPTH_PIXELS * 2) {
                        for (k = 0; k < DEPTH_PIXELS; k++) depth_px[k] = (uint16_t)(as->data[k * 2] | (as->data[k * 2 + 1] << 8));
                        ok = 1;
                    }
                    if (!ok) { n_bad++; break; }
                    n_depth++;
                    depth_bytes += as->info.total_len;
                    players_px = 0;
                    for (k = 0; k < DEPTH_PIXELS; k++) players_px += l2k_depth_player(depth_px[k]) != 0;
                    if (outdir) save_depth(outdir);
                } else {
                    n_color++;
                    color_bytes += as->info.total_len;
                    if (outdir && as->info.encoding == L2K_COLOR_JPEG) write_file(outdir, "color.jpg", NULL, as->data, as->info.total_len);
                }
                break;
            }
            case L2K_PKT_BYE:
                printf("sender said BYE\n");
                have_peer = 0;
                break;
            default:
                break;
            }
            continue;
        }
        /* nothing waiting */
        if (now - last_report >= 1000000) {
            int tracked = 0, first = -1;
            for (i = 0; i < L2K_SKELETON_SLOTS; i++) {
                if (have_skel && skel.skeletons[i].state == L2K_SKELETON_TRACKED) {
                    tracked++;
                    if (first < 0) first = i;
                }
            }
            if (!have_peer || now - last_packet > (uint64_t)L2K_PEER_TIMEOUT_MS * 1000) {
                printf("waiting for a sender...\n");
            } else {
                printf("skel %2u/s  depth %2u/s (%u B, %u player px)  colour %2u/s (%u B)  lost %u  bad %u  tracked %d",
                       n_skel, n_depth, n_depth ? depth_bytes / n_depth : 0, players_px, n_color, n_color ? color_bytes / n_color : 0, n_lost, n_bad, tracked);
                if (first >= 0) {
                    const l2k_skeleton *s = &skel.skeletons[first];
                    printf("  slot %d id %u head (%.2f %.2f %.2f) handL (%.2f %.2f %.2f) floor %.2f tilt %.1f", first, (unsigned)s->tracking_id,
                           s->joints[L2K_JOINT_HEAD].x, s->joints[L2K_JOINT_HEAD].y, s->joints[L2K_JOINT_HEAD].z,
                           s->joints[L2K_JOINT_HAND_LEFT].x, s->joints[L2K_JOINT_HAND_LEFT].y, s->joints[L2K_JOINT_HAND_LEFT].z,
                           skel.floor_plane.w, l2k_elevation_degrees(skel.up));
                }
                printf("\n");
            }
            fflush(stdout);
            n_skel = n_depth = n_color = n_lost = n_bad = depth_bytes = color_bytes = 0;
            last_report = now;
        }
        if (run_seconds > 0 && (double)(now - start) / 1e6 >= run_seconds) break;
        net_sleep_us(500);
    }
    net_close(sock);
    return 0;
}
