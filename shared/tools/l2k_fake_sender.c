/*
 * l2k_fake_sender - a Lidar2Kinect sender without a camera. Streams the synthetic dancing figure
 * (skeleton, capsule-figure depth image, and optionally one JPEG repeated as the colour image), so
 * receivers can be developed and tested on a PC alone. It follows the same HELLO / CONFIG rules as
 * the iOS app.
 *
 *   l2k_fake_sender [-p port] [-n players] [-f fps] [-j colour.jpg] [-t seconds] host
 */
#include "l2k/l2k.h"
#include "l2k_net.h"

#include <stdlib.h>

#define DEPTH_PIXELS (L2K_DEPTH_WIDTH * L2K_DEPTH_HEIGHT)

static uint16_t depth_px[DEPTH_PIXELS];
static uint8_t depth_enc[DEPTH_PIXELS * 3];
static uint8_t jpeg[L2K_MAX_CHUNKS * L2K_CHUNK_PAYLOAD];

static net_socket g_sock;
static struct sockaddr_in g_to;
static uint32_t g_session, g_seq;

static void send_packet(const uint8_t *pkt, size_t len) {
    if (len) sendto(g_sock, (const char *)pkt, (int)len, 0, (struct sockaddr *)&g_to, sizeof g_to);
}

static void send_image(uint8_t type, uint8_t encoding, uint16_t w, uint16_t h, const uint8_t *data, size_t len, uint64_t ts, uint32_t frame) {
    static uint8_t pkt[L2K_MAX_DATAGRAM];
    l2k_chunk c;
    uint32_t i, count = l2k_chunk_count(len);
    memset(&c, 0, sizeof c);
    c.timestamp_us = ts;
    c.frame_number = frame;
    c.width = w;
    c.height = h;
    c.encoding = encoding;
    for (i = 0; i < count; i++) {
        l2k_chunk_slice(&c, data, len, i);
        send_packet(pkt, l2k_encode_chunk(pkt, sizeof pkt, g_session, ++g_seq, type, &c));
    }
}

int main(int argc, char **argv) {
    uint16_t port = L2K_DEFAULT_PORT;
    int players = 1, fps = 30, i;
    const char *host = NULL, *jpeg_path = NULL;
    double run_seconds = 0;
    size_t jpeg_len = 0;
    l2k_mapper mapper;
    l2k_config cfg;
    uint64_t start, next_frame, last_hello = 0, last_config = 0, last_report;
    uint32_t frame = 0;
    unsigned sent = 0;
    int have_config = 0;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-p") && i + 1 < argc) port = (uint16_t)atoi(argv[++i]);
        else if (!strcmp(argv[i], "-n") && i + 1 < argc) players = atoi(argv[++i]) > 1 ? 2 : 1;
        else if (!strcmp(argv[i], "-f") && i + 1 < argc) fps = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-j") && i + 1 < argc) jpeg_path = argv[++i];
        else if (!strcmp(argv[i], "-t") && i + 1 < argc) run_seconds = atof(argv[++i]);
        else if (argv[i][0] != '-' && !host) host = argv[i];
        else host = NULL, i = argc;
    }
    if (!host || fps < 1 || fps > 120) {
        fprintf(stderr, "usage: l2k_fake_sender [-p port] [-n players] [-f fps] [-j colour.jpg] [-t seconds] host\n");
        return 2;
    }
    if (jpeg_path) {
        FILE *f = fopen(jpeg_path, "rb");
        if (f) {
            jpeg_len = fread(jpeg, 1, sizeof jpeg, f);
            fclose(f);
        }
        if (!jpeg_len) {
            fprintf(stderr, "cannot read %s\n", jpeg_path);
            return 1;
        }
    }
    if (!net_init() || (g_sock = net_udp(0)) == NET_INVALID) {
        fprintf(stderr, "cannot open a UDP socket\n");
        return 1;
    }
    memset(&g_to, 0, sizeof g_to);
    g_to.sin_family = AF_INET;
    g_to.sin_port = htons(port);
    if (inet_pton(AF_INET, host, &g_to.sin_addr) != 1) {
        fprintf(stderr, "host must be an IPv4 address\n");
        return 1;
    }
    l2k_mapper_init(&mapper, NULL);
    memset(&cfg, 0, sizeof cfg);
    start = next_frame = last_report = net_now_us();
    g_session = (uint32_t)(start ^ (start >> 32)) | 1u;
    printf("l2k_fake_sender: %d player(s) at %d fps to %s:%u\n", players, fps, host, (unsigned)port);
    fflush(stdout);

    for (;;) {
        static uint8_t pkt[2048];
        uint64_t now = net_now_us();
        int n;

        /* CONFIG from the receiver */
        while ((n = (int)recvfrom(g_sock, (char *)pkt, sizeof pkt, 0, NULL, NULL)) > 0) {
            l2k_config c;
            if (l2k_decode_config(pkt, (size_t)n, &c) == L2K_OK) {
                if (!have_config) printf("receiver \"%s\" wants streams 0x%x, round trip %.1f ms\n", c.name, (unsigned)c.streams, (double)(now - c.echo_sender_time_us) / 1000.0);
                cfg = c;
                have_config = 1;
                last_config = now;
            }
        }
        if (have_config && now - last_config > (uint64_t)L2K_PEER_TIMEOUT_MS * 1000) {
            printf("receiver silent, waiting\n");
            have_config = 0;
        }

        /* HELLO once a second, always */
        if (!last_hello || now - last_hello >= (uint64_t)L2K_HELLO_INTERVAL_MS * 1000) {
            l2k_hello h;
            memset(&h, 0, sizeof h);
            h.sender_time_us = now;
            h.caps = L2K_STREAM_SKELETON | L2K_STREAM_DEPTH | (jpeg_len ? L2K_STREAM_COLOR : 0);
            h.max_tracked = (uint8_t)players;
            h.source_kind = L2K_SOURCE_SYNTHETIC;
            strcpy(h.name, "l2k_fake_sender");
            send_packet(pkt, l2k_encode_hello(pkt, sizeof pkt, g_session, ++g_seq, &h));
            last_hello = now;
        }

        if (now >= next_frame) {
            next_frame += 1000000ull / (uint64_t)fps;
            if (next_frame < now) next_frame = now;
            if (have_config) {
                l2k_src_body bodies[2];
                l2k_skeleton_frame f;
                l2k_vec3 up = {L2K_SYNTH_UP_X, L2K_SYNTH_UP_Y, L2K_SYNTH_UP_Z};
                double t = (double)(now - start) / 1e6;
                for (i = 0; i < players; i++) l2k_synth_body(t, i, players, &bodies[i]);
                l2k_mapper_map(&mapper, bodies, players, up, l2k_quadrant_from_up(up.x, up.y), now, L2K_SOURCE_SYNTHETIC, &f);
                frame = f.frame_number;
                if (cfg.streams & L2K_STREAM_SKELETON) send_packet(pkt, l2k_encode_skeleton(pkt, sizeof pkt, g_session, ++g_seq, &f));
                if (cfg.streams & L2K_STREAM_DEPTH) {
                    size_t len;
                    l2k_depth_rasterize(&f, depth_px, L2K_DEPTH_WIDTH, L2K_DEPTH_HEIGHT, 0);
                    len = l2k_depth_d1_encode(depth_px, DEPTH_PIXELS, depth_enc, sizeof depth_enc);
                    send_image(L2K_PKT_DEPTH, L2K_DEPTH_D1, L2K_DEPTH_WIDTH, L2K_DEPTH_HEIGHT, depth_enc, len, now, frame);
                }
                if ((cfg.streams & L2K_STREAM_COLOR) && jpeg_len) {
                    send_image(L2K_PKT_COLOR, L2K_COLOR_JPEG, L2K_COLOR_WIDTH, L2K_COLOR_HEIGHT, jpeg, jpeg_len, now, frame);
                }
                sent++;
            }
        }
        if (now - last_report >= 5000000) {
            printf("%s, %u frames sent in 5 s\n", have_config ? "streaming" : "no receiver yet", sent);
            fflush(stdout);
            sent = 0;
            last_report = now;
        }
        if (run_seconds > 0 && (double)(now - start) / 1e6 >= run_seconds) break;
        net_sleep_us(1000);
    }
    {
        uint8_t bye[L2K_HEADER_SIZE];
        send_packet(bye, l2k_encode_bye(bye, sizeof bye, g_session, ++g_seq));
    }
    net_close(g_sock);
    return 0;
}
