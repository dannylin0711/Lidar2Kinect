/*
 * Lidar2Kinect shared library tests. Plain C, no framework:
 *   test_l2k [vectors_dir]            run the tests, compare against the golden packets
 *   test_l2k [vectors_dir] --write    (re)write the golden packets
 * Exit code 0 = all passed. The golden packets are built from integer arithmetic only, so every
 * compiler must produce and accept the same bytes.
 */
#include "l2k/l2k.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_checks = 0, g_failed = 0;
static const char *g_dir = "tests/vectors";
static int g_write = 0;

#define CHECK(cond) \
    do { \
        g_checks++; \
        if (!(cond)) { \
            g_failed++; \
            printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        } \
    } while (0)

static int nearly(float a, float b, float eps) {
    return fabsf(a - b) <= eps;
}

/* ---- golden packets ---- */

static void golden(const char *name, const uint8_t *data, size_t len) {
    char path[1024];
    FILE *f;
    snprintf(path, sizeof path, "%s/%s", g_dir, name);
    if (g_write) {
        f = fopen(path, "wb");
        if (!f || fwrite(data, 1, len, f) != len) {
            printf("FAIL cannot write %s\n", path);
            g_failed++;
        }
        if (f) fclose(f);
        return;
    }
    f = fopen(path, "rb");
    g_checks++;
    if (!f) {
        printf("FAIL missing golden file %s\n", path);
        g_failed++;
        return;
    }
    {
        static uint8_t buf[1 << 16];
        size_t n = fread(buf, 1, sizeof buf, f);
        fclose(f);
        if (n != len || memcmp(buf, data, len) != 0) {
            printf("FAIL golden mismatch %s (file %u bytes, built %u bytes)\n", path, (unsigned)n, (unsigned)len);
            g_failed++;
        }
    }
}

/* A frame made of exactly representable numbers. */
static void fixed_frame(l2k_skeleton_frame *f) {
    int j;
    l2k_skeleton *a, *b;
    memset(f, 0, sizeof *f);
    f->timestamp_us = 0x0001020304050607ull;
    f->frame_number = 1234;
    f->flags = L2K_FRAME_FLOOR_VALID;
    f->floor_plane.x = 0.0f;
    f->floor_plane.y = 1.0f;
    f->floor_plane.z = 0.0f;
    f->floor_plane.w = 1.25f;
    f->up.x = 0.0f;
    f->up.y = 1.0f;
    f->up.z = 0.0f;
    f->source_kind = L2K_SOURCE_SYNTHETIC;
    a = &f->skeletons[1];
    a->state = L2K_SKELETON_TRACKED;
    a->quality = L2K_QUALITY_CLIPPED_BOTTOM;
    a->tracking_id = 77;
    a->position.x = -0.5f;
    a->position.y = 0.25f;
    a->position.z = 2.5f;
    for (j = 0; j < L2K_JOINT_COUNT; j++) {
        a->joints[j].x = (float)(j - 10) / 64.0f;
        a->joints[j].y = (float)(j * 3 - 20) / 32.0f;
        a->joints[j].z = 2.0f + (float)j / 128.0f;
        a->joint_state[j] = (uint8_t)(j % 3);
    }
    b = &f->skeletons[4];
    b->state = L2K_SKELETON_POSITION_ONLY;
    b->tracking_id = 0x80000001u;
    b->position.x = 1.0f;
    b->position.y = -0.125f;
    b->position.z = 3.75f;
}

/* A depth image made by integer arithmetic: background 0, a block of player 1 with a depth ramp,
 * a block of player 2 with large depth steps (forces literals). */
static void fixed_depth(uint16_t *px, int w, int h) {
    int x, y;
    for (y = 0; y < h; y++) {
        for (x = 0; x < w; x++) {
            uint16_t v = 0;
            if (x >= 40 && x < 120 && y >= 30 && y < 200) v = l2k_depth_pack((uint32_t)(2000 + (x - 40) * 2 + (y & 3)), 1);
            if (x >= 200 && x < 260 && y >= 60 && y < 220) v = l2k_depth_pack((uint32_t)(1500 + ((x * 37 + y * 11) % 900)), 2);
            px[y * w + x] = v;
        }
    }
}

static void test_control_packets(void) {
    uint8_t buf[256];
    l2k_hello h, h2;
    l2k_config c, c2;
    l2k_header hd;
    size_t n;

    memset(&h, 0, sizeof h);
    h.sender_time_us = 123456789012345ull;
    h.caps = L2K_STREAM_SKELETON | L2K_STREAM_DEPTH | L2K_STREAM_COLOR;
    h.max_tracked = 1;
    h.source_kind = L2K_SOURCE_ARKIT_BODY;
    strcpy(h.name, "Test iPhone");
    n = l2k_encode_hello(buf, sizeof buf, 0xAABBCCDDu, 7, &h);
    CHECK(n == 64);
    golden("hello.bin", buf, n);
    CHECK(l2k_decode_header(buf, n, &hd) == L2K_OK);
    CHECK(hd.type == L2K_PKT_HELLO && hd.length == 64 && hd.session == 0xAABBCCDDu && hd.seq == 7);
    CHECK(buf[0] == 'L' && buf[1] == '2' && buf[2] == 'K' && buf[3] == '1');
    CHECK(l2k_decode_hello(buf, n, &h2) == L2K_OK);
    CHECK(h2.sender_time_us == h.sender_time_us && h2.caps == h.caps && h2.max_tracked == 1);
    CHECK(h2.source_kind == L2K_SOURCE_ARKIT_BODY && strcmp(h2.name, "Test iPhone") == 0);
    CHECK(l2k_decode_hello(buf, n - 1, &h2) == L2K_ERR_SHORT);
    CHECK(l2k_decode_config(buf, n, &c2) == L2K_ERR_TYPE);
    CHECK(l2k_encode_hello(buf, 63, 1, 1, &h) == 0);

    memset(&c, 0, sizeof c);
    c.echo_sender_time_us = h.sender_time_us;
    c.receiver_time_us = 999;
    c.streams = L2K_STREAM_SKELETON | L2K_STREAM_DEPTH;
    c.skeleton_flags = 2;
    c.consumer_kind = L2K_CONSUMER_FAKE_KINECT;
    c.depth_mode = L2K_DEPTH_MODE_FULL;
    c.color_quality = 60;
    strcpy(c.name, "0123456789abcdef0123456789abcdef"); /* 32 characters: fills the field */
    n = l2k_encode_config(buf, sizeof buf, 0x01020304u, 0xFFFFFFFFu, &c);
    CHECK(n == 76);
    golden("config.bin", buf, n);
    CHECK(l2k_decode_config(buf, n, &c2) == L2K_OK);
    CHECK(c2.echo_sender_time_us == c.echo_sender_time_us && c2.receiver_time_us == 999);
    CHECK(c2.streams == c.streams && c2.skeleton_flags == 2 && c2.consumer_kind == L2K_CONSUMER_FAKE_KINECT);
    CHECK(c2.depth_mode == L2K_DEPTH_MODE_FULL && c2.color_quality == 60);
    CHECK(strcmp(c2.name, c.name) == 0);

    n = l2k_encode_bye(buf, sizeof buf, 5, 6);
    CHECK(n == 16);
    golden("bye.bin", buf, n);
    CHECK(l2k_decode_header(buf, n, &hd) == L2K_OK && hd.type == L2K_PKT_BYE);

    buf[0] ^= 1;
    CHECK(l2k_decode_header(buf, n, &hd) == L2K_ERR_MAGIC);
    buf[0] ^= 1;
    buf[4] = 9;
    CHECK(l2k_decode_header(buf, n, &hd) == L2K_ERR_VERSION);
}

static void test_skeleton_packet(void) {
    static uint8_t buf[L2K_MAX_DATAGRAM];
    l2k_skeleton_frame f, g;
    size_t n;
    int i;

    fixed_frame(&f);
    n = l2k_encode_skeleton(buf, sizeof buf, 0x11223344u, 42, &f);
    CHECK(n == 16 + 48 + 20 + 260 + 20);
    golden("skeleton.bin", buf, n);
    CHECK(l2k_decode_skeleton(buf, n, &g) == L2K_OK);
    CHECK(memcmp(&f, &g, sizeof f) == 0);
    for (i = 16; i < (int)n; i += 7) CHECK(l2k_decode_skeleton(buf, (size_t)i, &g) != L2K_OK);

    /* four tracked bodies still fit one datagram; six do not */
    for (i = 0; i < 4; i++) f.skeletons[i] = f.skeletons[1];
    f.skeletons[5] = f.skeletons[4];
    n = l2k_encode_skeleton(buf, sizeof buf, 1, 1, &f);
    CHECK(n > 0 && n <= L2K_MAX_DATAGRAM);
    CHECK(l2k_decode_skeleton(buf, n, &g) == L2K_OK && memcmp(&f, &g, sizeof f) == 0);
    for (i = 0; i < 6; i++) f.skeletons[i] = f.skeletons[1];
    CHECK(l2k_encode_skeleton(buf, sizeof buf, 1, 1, &f) == 0);
}

static void test_depth_codec(void) {
    enum { W = L2K_DEPTH_WIDTH, H = L2K_DEPTH_HEIGHT, N = W * H };
    static uint16_t px[N], back[N];
    static uint8_t enc[N * 3];
    size_t n, i;
    uint32_t seed = 12345;

    fixed_depth(px, W, H);
    n = l2k_depth_d1_encode(px, N, enc, sizeof enc);
    CHECK(n > 0 && n < N); /* far smaller than raw (2N) */
    golden("depth_d1.bin", enc, n);
    CHECK(l2k_depth_d1_decode(enc, n, back, N) == 0);
    CHECK(memcmp(px, back, sizeof px) == 0);
    CHECK(l2k_depth_d1_decode(enc, n - 1, back, N) != 0);
    CHECK(l2k_depth_d1_decode(enc, n, back, N - 1) != 0);
    CHECK(l2k_depth_d1_encode(px, N, enc, n - 1) == 0);

    /* noise: worst case stays within the bound and round-trips */
    for (i = 0; i < N; i++) {
        seed = seed * 1664525u + 1013904223u;
        px[i] = (uint16_t)(seed >> 16);
    }
    n = l2k_depth_d1_encode(px, N, enc, sizeof enc);
    CHECK(n > 0 && n <= l2k_depth_d1_bound(N));
    CHECK(l2k_depth_d1_decode(enc, n, back, N) == 0 && memcmp(px, back, sizeof px) == 0);

    /* all zero: 600 run bytes */
    memset(px, 0, sizeof px);
    n = l2k_depth_d1_encode(px, N, enc, sizeof enc);
    CHECK(n == N / 128);
    CHECK(l2k_depth_d1_decode(enc, n, back, N) == 0 && memcmp(px, back, sizeof px) == 0);
}

static void test_chunks(void) {
    enum { W = L2K_DEPTH_WIDTH, H = L2K_DEPTH_HEIGHT, N = W * H };
    static uint16_t px[N];
    static uint8_t enc[N * 3], store[N * 3], pkt[64][L2K_MAX_DATAGRAM];
    size_t pkt_len[64];
    l2k_assembler as;
    l2k_chunk c, d;
    size_t n;
    uint32_t count, i;
    int r = 0;

    fixed_depth(px, W, H);
    n = l2k_depth_d1_encode(px, N, enc, sizeof enc);
    count = l2k_chunk_count(n);
    CHECK(count >= 2 && count <= 64);
    memset(&c, 0, sizeof c);
    c.timestamp_us = 5000;
    c.frame_number = 10;
    c.width = W;
    c.height = H;
    c.encoding = L2K_DEPTH_D1;
    for (i = 0; i < count; i++) {
        CHECK(l2k_chunk_slice(&c, enc, n, i) == 1);
        pkt_len[i] = l2k_encode_chunk(pkt[i], sizeof pkt[i], 9, 100 + i, L2K_PKT_DEPTH, &c);
        CHECK(pkt_len[i] == (size_t)L2K_HEADER_SIZE + L2K_CHUNK_HEADER_SIZE + c.chunk_len);
    }
    CHECK(l2k_chunk_slice(&c, enc, n, count) == 0);
    golden("depth_chunk0.bin", pkt[0], pkt_len[0]);

    /* out of order, with a duplicate */
    l2k_assembler_init(&as, store, sizeof store);
    for (i = 0; i < count; i++) {
        uint32_t k = (i * 7 + 3) % count; /* a permutation when gcd(7,count)=1; duplicates are harmless otherwise */
        CHECK(l2k_decode_chunk(pkt[k], pkt_len[k], &d) == L2K_OK);
        r = l2k_assembler_push(&as, &d);
        CHECK(r >= 0);
        if (i == 0) CHECK(l2k_assembler_push(&as, &d) == 0);
    }
    for (i = 0; i < count && !as.complete; i++) {
        CHECK(l2k_decode_chunk(pkt[i], pkt_len[i], &d) == L2K_OK);
        r = l2k_assembler_push(&as, &d);
    }
    CHECK(as.complete && as.info.total_len == n && as.info.frame_number == 10);
    CHECK(memcmp(store, enc, n) == 0);
    CHECK(as.frames_completed == 1 && as.frames_dropped == 0);

    /* an older frame is refused; a newer frame replaces an unfinished one */
    c.frame_number = 9;
    l2k_chunk_slice(&c, enc, n, 0);
    CHECK(l2k_assembler_push(&as, &c) == L2K_ERR_STALE);
    c.frame_number = 11;
    l2k_chunk_slice(&c, enc, n, 0);
    CHECK(l2k_assembler_push(&as, &c) == 0 && !as.complete);
    c.frame_number = 12;
    for (i = 0; i < count; i++) {
        l2k_chunk_slice(&c, enc, n, i);
        r = l2k_assembler_push(&as, &c);
    }
    CHECK(r == 1 && as.complete && as.info.frame_number == 12);
    CHECK(as.frames_completed == 2 && as.frames_dropped == 1);

    /* a truncated chunk packet is rejected */
    CHECK(l2k_decode_chunk(pkt[0], pkt_len[0] - 1, &d) != L2K_OK);
}

static void test_mapping(void) {
    l2k_mapper m;
    l2k_src_body body[2], rolled;
    l2k_skeleton_frame f, g;
    l2k_vec3 up = {L2K_SYNTH_UP_X, L2K_SYNTH_UP_Y, L2K_SYNTH_UP_Z};
    const l2k_skeleton *s;
    int i, q;

    l2k_mapper_init(&m, NULL);
    for (i = 0; i < 30; i++) {
        l2k_synth_body(i / 30.0, 0, 1, &body[0]);
        l2k_mapper_map(&m, body, 1, up, l2k_quadrant_from_up(up.x, up.y), (uint64_t)i * 33333, L2K_SOURCE_SYNTHETIC, &f);
    }
    s = &f.skeletons[0];
    CHECK(f.frame_number == 30);
    CHECK(s->state == L2K_SKELETON_TRACKED && s->tracking_id == 1);
    CHECK(f.skeletons[1].state == L2K_SKELETON_NOT_TRACKED);
    /* in front of the sensor, upright */
    CHECK(s->position.z > 2.0f && s->position.z < 3.0f);
    CHECK(s->joints[L2K_JOINT_HEAD].y > s->joints[L2K_JOINT_SHOULDER_CENTER].y);
    CHECK(s->joints[L2K_JOINT_SHOULDER_CENTER].y > s->joints[L2K_JOINT_SPINE].y);
    CHECK(s->joints[L2K_JOINT_SPINE].y > s->joints[L2K_JOINT_HIP_CENTER].y);
    CHECK(s->joints[L2K_JOINT_HIP_LEFT].y > s->joints[L2K_JOINT_KNEE_LEFT].y);
    CHECK(s->joints[L2K_JOINT_KNEE_LEFT].y > s->joints[L2K_JOINT_ANKLE_LEFT].y);
    /* the player faces the sensor: the player's RIGHT side is at +x (the sensor's left) */
    CHECK(s->joints[L2K_JOINT_SHOULDER_RIGHT].x > s->joints[L2K_JOINT_SHOULDER_LEFT].x);
    CHECK(s->joints[L2K_JOINT_HIP_RIGHT].x > s->joints[L2K_JOINT_HIP_LEFT].x);
    for (i = 0; i < L2K_JOINT_COUNT; i++) CHECK(s->joint_state[i] == L2K_JOINT_TRACKED);
    /* floor: the synthetic camera is 1 m up and level */
    CHECK(f.flags & L2K_FRAME_FLOOR_VALID);
    CHECK(nearly(f.floor_plane.y, 1.0f, 1e-5f) && nearly(f.floor_plane.w, L2K_SYNTH_CAMERA_HEIGHT, 0.06f));
    CHECK(nearly(l2k_elevation_degrees(f.up), 0.0f, 0.01f));
    {
        l2k_vec3 tilted = {0.0f, 0.8660254f, 0.5f}; /* up as seen by a sensor looking 30 degrees upwards */
        CHECK(nearly(l2k_elevation_degrees(tilted), 30.0f, 0.01f));
    }

    /* the same scene seen by a rolled device gives the same skeleton */
    l2k_synth_body(0.7, 0, 1, &body[0]);
    l2k_mapper_init(&m, NULL);
    l2k_mapper_map(&m, body, 1, up, 0, 0, L2K_SOURCE_SYNTHETIC, &f);
    for (q = 1; q < 4; q++) {
        l2k_vec3 up_q;
        rolled = body[0];
        for (i = 0; i < L2K_SRC_JOINT_COUNT; i++) {
            l2k_vec3 p = body[0].p[i], r;
            r.z = p.z;
            if (q == 1) { r.x = p.y; r.y = -p.x; }
            else if (q == 2) { r.x = -p.x; r.y = -p.y; }
            else { r.x = -p.y; r.y = p.x; }
            rolled.p[i] = r;
        }
        up_q.z = 0.0f;
        up_q.x = q == 1 ? 1.0f : q == 3 ? -1.0f : 0.0f;
        up_q.y = q == 2 ? -1.0f : 0.0f;
        CHECK(l2k_quadrant_from_up(up_q.x, up_q.y) == q);
        l2k_mapper_init(&m, NULL);
        l2k_mapper_map(&m, &rolled, 1, up_q, q, 0, L2K_SOURCE_SYNTHETIC, &g);
        for (i = 0; i < L2K_JOINT_COUNT; i++) {
            CHECK(nearly(f.skeletons[0].joints[i].x, g.skeletons[0].joints[i].x, 1e-5f));
            CHECK(nearly(f.skeletons[0].joints[i].y, g.skeletons[0].joints[i].y, 1e-5f));
            CHECK(nearly(f.skeletons[0].joints[i].z, g.skeletons[0].joints[i].z, 1e-5f));
        }
        CHECK(nearly(g.up.y, 1.0f, 1e-5f));
    }

    /* two bodies keep their slots when the order changes and when one leaves */
    l2k_mapper_init(&m, NULL);
    l2k_synth_body(0.0, 0, 2, &body[0]);
    l2k_synth_body(0.0, 1, 2, &body[1]);
    l2k_mapper_map(&m, body, 2, up, 0, 0, L2K_SOURCE_SYNTHETIC, &f);
    CHECK(f.skeletons[0].tracking_id == 1 && f.skeletons[1].tracking_id == 2);
    CHECK(f.skeletons[0].position.x > f.skeletons[1].position.x); /* player 0 stands at image left = camera -x = Kinect +x */
    l2k_synth_body(0.1, 1, 2, &body[0]);
    l2k_synth_body(0.1, 0, 2, &body[1]);
    l2k_mapper_map(&m, body, 2, up, 0, 0, L2K_SOURCE_SYNTHETIC, &f);
    CHECK(f.skeletons[0].tracking_id == 1 && f.skeletons[1].tracking_id == 2);
    l2k_mapper_map(&m, body, 1, up, 0, 0, L2K_SOURCE_SYNTHETIC, &f); /* only id 2 remains */
    CHECK(f.skeletons[0].state == L2K_SKELETON_NOT_TRACKED && f.skeletons[1].tracking_id == 2);

    /* missing optional joints are derived and flagged inferred; too little data = position only */
    l2k_synth_body(0.3, 0, 1, &body[0]);
    body[0].conf[L2K_SRC_HAND_LEFT] = 0;
    body[0].conf[L2K_SRC_TOE_RIGHT] = 0;
    body[0].conf[L2K_SRC_PELVIS] = 0;
    l2k_mapper_init(&m, NULL);
    l2k_mapper_map(&m, body, 1, up, 0, 0, L2K_SOURCE_SYNTHETIC, &f);
    s = &f.skeletons[0];
    CHECK(s->state == L2K_SKELETON_TRACKED);
    CHECK(s->joint_state[L2K_JOINT_HAND_LEFT] == L2K_JOINT_INFERRED && s->joint_state[L2K_JOINT_HAND_RIGHT] == L2K_JOINT_TRACKED);
    CHECK(s->joint_state[L2K_JOINT_FOOT_RIGHT] == L2K_JOINT_INFERRED);
    CHECK(s->joints[L2K_JOINT_FOOT_RIGHT].z < s->joints[L2K_JOINT_ANKLE_RIGHT].z); /* toes point at the sensor */
    body[0].conf[L2K_SRC_NECK] = 0;
    l2k_mapper_map(&m, body, 1, up, 0, 0, L2K_SOURCE_SYNTHETIC, &f);
    CHECK(f.skeletons[0].state == L2K_SKELETON_POSITION_ONLY && f.skeletons[0].position.z > 2.0f);

    /* a joint the source loses is inferred, never "not tracked": never seen = on its parent,
     * seen before = at its last offset from the parent, which follows the parent */
    l2k_synth_body(0.3, 0, 1, &body[0]);
    body[0].conf[L2K_SRC_ELBOW_LEFT] = 0;
    l2k_mapper_init(&m, NULL);
    l2k_mapper_map(&m, body, 1, up, 0, 0, L2K_SOURCE_SYNTHETIC, &f);
    s = &f.skeletons[0];
    CHECK(s->state == L2K_SKELETON_TRACKED && s->joint_state[L2K_JOINT_ELBOW_LEFT] == L2K_JOINT_INFERRED);
    CHECK(nearly(s->joints[L2K_JOINT_ELBOW_LEFT].x, s->joints[L2K_JOINT_SHOULDER_LEFT].x, 1e-6f));
    CHECK(nearly(s->joints[L2K_JOINT_ELBOW_LEFT].y, s->joints[L2K_JOINT_SHOULDER_LEFT].y, 1e-6f));
    for (i = 0; i < L2K_JOINT_COUNT; i++) CHECK(s->joint_state[i] != L2K_JOINT_NOT_TRACKED);
    {
        l2k_vec3 seen, shoulder;
        l2k_synth_body(0.3, 0, 1, &body[0]);
        l2k_mapper_init(&m, NULL);
        l2k_mapper_map(&m, body, 1, up, 0, 0, L2K_SOURCE_SYNTHETIC, &f);
        seen = f.skeletons[0].joints[L2K_JOINT_ELBOW_LEFT];
        shoulder = f.skeletons[0].joints[L2K_JOINT_SHOULDER_LEFT];
        for (i = 0; i < L2K_SRC_JOINT_COUNT; i++) body[0].p[i].x += 0.5f; /* the whole body moves sideways */
        body[0].conf[L2K_SRC_ELBOW_LEFT] = 0;
        l2k_mapper_map(&m, body, 1, up, 0, 0, L2K_SOURCE_SYNTHETIC, &f);
        s = &f.skeletons[0];
        CHECK(s->joint_state[L2K_JOINT_ELBOW_LEFT] == L2K_JOINT_INFERRED);
        CHECK(nearly(s->joints[L2K_JOINT_ELBOW_LEFT].x - s->joints[L2K_JOINT_SHOULDER_LEFT].x, seen.x - shoulder.x, 1e-5f));
        CHECK(nearly(s->joints[L2K_JOINT_ELBOW_LEFT].y - s->joints[L2K_JOINT_SHOULDER_LEFT].y, seen.y - shoulder.y, 1e-5f));
        CHECK(nearly(s->joints[L2K_JOINT_ELBOW_LEFT].x, seen.x - 0.5f, 1e-5f)); /* camera +x is Kinect -x */
    }
}

static void test_upper_body(void) {
    static const uint8_t LOWER[9] = {L2K_SRC_PELVIS, L2K_SRC_HIP_LEFT, L2K_SRC_KNEE_LEFT, L2K_SRC_ANKLE_LEFT, L2K_SRC_TOE_LEFT,
                                     L2K_SRC_HIP_RIGHT, L2K_SRC_KNEE_RIGHT, L2K_SRC_ANKLE_RIGHT, L2K_SRC_TOE_RIGHT};
    static const uint8_t ARMS[10] = {L2K_JOINT_SHOULDER_CENTER, L2K_JOINT_HEAD, L2K_JOINT_SHOULDER_LEFT, L2K_JOINT_ELBOW_LEFT,
                                     L2K_JOINT_WRIST_LEFT, L2K_JOINT_HAND_LEFT, L2K_JOINT_SHOULDER_RIGHT, L2K_JOINT_ELBOW_RIGHT,
                                     L2K_JOINT_WRIST_RIGHT, L2K_JOINT_HAND_RIGHT};
    l2k_mapper m;
    l2k_map_params prm;
    l2k_src_body full, upper;
    l2k_skeleton_frame ref, f;
    l2k_vec3 up = {0, 1, 0};
    const l2k_skeleton *s;
    int i, side;
    float width;

    l2k_synth_body(0.6, 0, 1, &full);
    upper = full;
    for (i = 0; i < 9; i++) upper.conf[LOWER[i]] = 0; /* the camera sees nothing below the chest */

    /* reference: the whole body, normal mode */
    l2k_mapper_init(&m, NULL);
    l2k_mapper_map(&m, &full, 1, up, 0, 0, L2K_SOURCE_SYNTHETIC, &ref);
    CHECK(!(ref.flags & L2K_FRAME_UPPER_BODY));
    /* normal mode cannot track half a body */
    l2k_mapper_init(&m, NULL);
    l2k_mapper_map(&m, &upper, 1, up, 0, 0, L2K_SOURCE_SYNTHETIC, &f);
    CHECK(f.skeletons[0].state == L2K_SKELETON_POSITION_ONLY);

    /* upper-body mode tracks it, with a complete skeleton */
    l2k_map_params_default(&prm);
    prm.upper_body = 1;
    l2k_mapper_init(&m, &prm);
    l2k_mapper_map(&m, &upper, 1, up, 0, 0, L2K_SOURCE_SYNTHETIC, &f);
    s = &f.skeletons[0];
    CHECK(f.flags & L2K_FRAME_UPPER_BODY);
    CHECK(!(f.flags & L2K_FRAME_FLOOR_VALID)); /* no real feet, so no floor estimate */
    CHECK(s->state == L2K_SKELETON_TRACKED);
    for (i = 0; i < L2K_JOINT_COUNT; i++) CHECK(s->joint_state[i] != L2K_JOINT_NOT_TRACKED);
    /* the arms, head and shoulders are the player's own, unchanged */
    for (i = 0; i < 10; i++) {
        CHECK(s->joint_state[ARMS[i]] == L2K_JOINT_TRACKED);
        CHECK(nearly(s->joints[ARMS[i]].x, ref.skeletons[0].joints[ARMS[i]].x, 1e-6f));
        CHECK(nearly(s->joints[ARMS[i]].y, ref.skeletons[0].joints[ARMS[i]].y, 1e-6f));
        CHECK(nearly(s->joints[ARMS[i]].z, ref.skeletons[0].joints[ARMS[i]].z, 1e-6f));
    }
    /* the lower body is a standing pose straight below the neck, all inferred */
    width = s->joints[L2K_JOINT_SHOULDER_RIGHT].x - s->joints[L2K_JOINT_SHOULDER_LEFT].x;
    CHECK(width > 0.3f && width < 0.5f);
    CHECK(s->joint_state[L2K_JOINT_HIP_CENTER] == L2K_JOINT_INFERRED && s->joint_state[L2K_JOINT_SPINE] == L2K_JOINT_INFERRED);
    CHECK(nearly(s->joints[L2K_JOINT_HIP_CENTER].x, s->joints[L2K_JOINT_SHOULDER_CENTER].x, 1e-5f));
    CHECK(s->joints[L2K_JOINT_HIP_CENTER].y < s->joints[L2K_JOINT_SPINE].y);
    CHECK(s->joints[L2K_JOINT_SPINE].y < s->joints[L2K_JOINT_SHOULDER_CENTER].y);
    for (side = 0; side < 2; side++) {
        int hip = side ? L2K_JOINT_HIP_RIGHT : L2K_JOINT_HIP_LEFT;
        for (i = 0; i < 4; i++) CHECK(s->joint_state[hip + i] == L2K_JOINT_INFERRED);
        CHECK(nearly(s->joints[hip + 1].x, s->joints[hip].x, 1e-5f) && nearly(s->joints[hip + 2].x, s->joints[hip].x, 1e-5f));
        CHECK(s->joints[hip].y > s->joints[hip + 1].y && s->joints[hip + 1].y > s->joints[hip + 2].y);
        CHECK(nearly(s->joints[hip].y - s->joints[hip + 2].y, 1.60f * 1.40f * width, 1e-4f)); /* leg = 1.6 x torso, torso = 1.4 x shoulders */
        CHECK(s->joints[hip + 3].z < s->joints[hip + 2].z); /* toes towards the sensor */
    }
    CHECK(s->joints[L2K_JOINT_HIP_RIGHT].x > s->joints[L2K_JOINT_HIP_LEFT].x); /* same side as the shoulders */

    /* legs the source does report are ignored in this mode: a lifted knee stays a standing leg */
    l2k_mapper_init(&m, &prm);
    l2k_mapper_map(&m, &full, 1, up, 0, 0, L2K_SOURCE_SYNTHETIC, &f);
    s = &f.skeletons[0];
    CHECK(s->state == L2K_SKELETON_TRACKED && s->joint_state[L2K_JOINT_KNEE_LEFT] == L2K_JOINT_INFERRED);
    CHECK(nearly(s->joints[L2K_JOINT_KNEE_LEFT].z, s->joints[L2K_JOINT_HIP_LEFT].z, 1e-5f));
    CHECK(nearly(s->joints[L2K_JOINT_KNEE_RIGHT].z, s->joints[L2K_JOINT_HIP_RIGHT].z, 1e-5f));
    /* ...but a pelvis the source does see is used */
    CHECK(s->joint_state[L2K_JOINT_HIP_CENTER] == L2K_JOINT_TRACKED);
    CHECK(nearly(s->joints[L2K_JOINT_HIP_CENTER].y, ref.skeletons[0].joints[L2K_JOINT_HIP_CENTER].y, 1e-6f));

    /* without shoulders there is nothing to build on */
    upper.conf[L2K_SRC_SHOULDER_LEFT] = 0;
    l2k_mapper_init(&m, &prm);
    l2k_mapper_map(&m, &upper, 1, up, 0, 0, L2K_SOURCE_SYNTHETIC, &f);
    CHECK(f.skeletons[0].state == L2K_SKELETON_POSITION_ONLY);
}

static void test_virtual_camera(void) {
    /* a source camera wider than the virtual Kinect, off-centre principal point */
    l2k_src_camera cam = {0.78f, 1.04f, 0.51f, 0.49f};
    l2k_src_body body;
    int q, i;

    l2k_synth_body(0.4, 0, 1, &body);
    for (q = 0; q < 4; q++) {
        for (i = 0; i < L2K_SRC_JOINT_COUNT; i++) {
            /* project the camera-space point straight into the source image... */
            l2k_vec3 p = body.p[i], k;
            float sx = cam.cxn + cam.fxn * (p.x / -p.z), sy = cam.cyn - cam.fyn * (p.y / -p.z);
            float u, v, tx, ty;
            /* ...and through the virtual Kinect image: both must agree */
            k = l2k_camera_to_kinect(p, q);
            CHECK(nearly(k.z, -p.z, 1e-6f));
            CHECK(l2k_skeleton_to_depth_pixel(k, 320, 240, &u, &v) == 1);
            l2k_virtual_to_source(&cam, q, u / 320.0f, v / 240.0f, &tx, &ty);
            CHECK(nearly(sx, tx, 1e-4f) && nearly(sy, ty, 1e-4f));
        }
    }
    CHECK(l2k_quadrant_from_up(0.1f, 0.9f) == 0 && l2k_quadrant_from_up(0.9f, 0.1f) == 1);
    CHECK(l2k_quadrant_from_up(-0.1f, -0.9f) == 2 && l2k_quadrant_from_up(-0.9f, 0.1f) == 3);
    {
        l2k_vec3 behind = {0, 0, -1};
        float u, v;
        CHECK(l2k_skeleton_to_depth_pixel(behind, 320, 240, &u, &v) == 0);
    }
}

static void test_depth_images(void) {
    enum { W = L2K_DEPTH_WIDTH, H = L2K_DEPTH_HEIGHT };
    static uint16_t img[W * H];
    static float depth[64 * 48];
    static uint8_t mask[64 * 48];
    l2k_depth_source src;
    /* a source camera identical to the virtual Kinect */
    l2k_src_camera cam = {L2K_FOCAL_320 / 320.0f, L2K_FOCAL_320 / 240.0f, 0.5f, 0.5f};
    l2k_mapper m;
    l2k_src_body body[2];
    l2k_skeleton_frame f;
    l2k_vec3 up = {0, 1, 0};
    int x, y, n1 = 0, n2 = 0;
    float u, v;

    /* source: 2 m everywhere, a person mask on the LEFT third of the source image */
    for (y = 0; y < 48; y++) {
        for (x = 0; x < 64; x++) {
            depth[y * 64 + x] = 2.0f;
            mask[y * 64 + x] = (x < 20 && y >= 8) ? 255 : 0;
        }
    }
    depth[0] = 0.3f;  /* too near */
    depth[1] = 9.0f;  /* too far */
    memset(&src, 0, sizeof src);
    src.depth_m = depth;
    src.depth_w = 64;
    src.depth_h = 48;
    src.depth_stride = 64 * sizeof(float);
    src.mask = mask;
    src.mask_w = 64;
    src.mask_h = 48;
    src.mask_stride = 64;
    src.mask_threshold = 128;
    src.player_index = 3;
    l2k_depth_render(&src, &cam, 0, img, W, H);
    /* Kinect images are mirrored: the mask shows on the RIGHT */
    CHECK(l2k_depth_player(img[120 * W + 300]) == 3 && l2k_depth_mm(img[120 * W + 300]) == 2000);
    CHECK(l2k_depth_player(img[120 * W + 20]) == 0 && l2k_depth_mm(img[120 * W + 20]) == 2000);
    CHECK(l2k_depth_player(img[10 * W + 300]) == 0); /* above the mask */
    CHECK(l2k_depth_mm(img[0 * W + (W - 1)]) == 0);  /* source pixel 0 (too near) lands top-right */
    CHECK(l2k_depth_mm(img[0 * W + (W - 7)]) == 0);  /* source pixel 1 (too far) */
    src.players_only = 1;
    l2k_depth_render(&src, &cam, 0, img, W, H);
    CHECK(img[120 * W + 20] == 0 && l2k_depth_mm(img[120 * W + 300]) == 2000);
    /* no depth map: player pixels take the fallback depth */
    src.depth_m = NULL;
    src.fallback_depth_m = 2.5f;
    l2k_depth_render(&src, &cam, 0, img, W, H);
    CHECK(img[120 * W + 300] == l2k_depth_pack(2500, 3) && img[120 * W + 20] == 0);
    /* upside-down device: the mask moves to the left and to the top */
    src.depth_m = depth;
    l2k_depth_render(&src, &cam, 2, img, W, H);
    CHECK(l2k_depth_player(img[120 * W + 20]) == 3 && l2k_depth_player(img[120 * W + 300]) == 0);
    CHECK(l2k_depth_player(img[10 * W + 20]) == 3 && l2k_depth_player(img[230 * W + 20]) == 0);

    /* capsule figures: two players, each labelled with slot + 1, heads where the skeleton says */
    l2k_mapper_init(&m, NULL);
    l2k_synth_body(0.2, 0, 2, &body[0]);
    l2k_synth_body(0.2, 1, 2, &body[1]);
    l2k_mapper_map(&m, body, 2, up, 0, 0, L2K_SOURCE_SYNTHETIC, &f);
    l2k_depth_rasterize(&f, img, W, H, 0);
    for (x = 0; x < W * H; x++) {
        if (l2k_depth_player(img[x]) == 1) n1++;
        if (l2k_depth_player(img[x]) == 2) n2++;
    }
    CHECK(n1 > 1500 && n2 > 1500 && n1 + n2 < W * H / 2);
    CHECK(l2k_skeleton_to_depth_pixel(f.skeletons[0].joints[L2K_JOINT_HEAD], W, H, &u, &v) == 1);
    CHECK(l2k_depth_player(img[(int)v * W + (int)u]) == 1);
    CHECK(abs((int)l2k_depth_mm(img[(int)v * W + (int)u]) - (int)(f.skeletons[0].joints[L2K_JOINT_HEAD].z * 1000.0f)) < 20);
    CHECK(l2k_skeleton_to_depth_pixel(f.skeletons[1].joints[L2K_JOINT_SPINE], W, H, &u, &v) == 1);
    CHECK(l2k_depth_player(img[(int)v * W + (int)u]) == 2);
    CHECK(img[0] == 0);
}

static void test_smoothing(void) {
    l2k_smoother sm;
    l2k_smooth_params p;
    l2k_mapper m;
    l2k_src_body body;
    l2k_skeleton_frame f, raw;
    l2k_vec3 up = {0, 1, 0};
    int i, j;
    float worst = 0.0f;

    l2k_smooth_params_default(&p);
    CHECK(p.smoothing == 0.5f && p.correction == 0.5f && p.prediction == 0.5f);
    CHECK(p.jitter_radius == 0.05f && p.max_deviation_radius == 0.04f);

    /* a still body is left where it is */
    l2k_smoother_init(&sm);
    l2k_mapper_init(&m, NULL);
    l2k_synth_body(0.0, 0, 1, &body);
    for (i = 0; i < 10; i++) {
        l2k_mapper_map(&m, &body, 1, up, 0, 0, L2K_SOURCE_SYNTHETIC, &f);
        raw = f;
        l2k_smoother_apply(&sm, &f, &p);
    }
    for (j = 0; j < L2K_JOINT_COUNT; j++) {
        CHECK(nearly(f.skeletons[0].joints[j].x, raw.skeletons[0].joints[j].x, 1e-5f));
        CHECK(nearly(f.skeletons[0].joints[j].y, raw.skeletons[0].joints[j].y, 1e-5f));
        CHECK(nearly(f.skeletons[0].joints[j].z, raw.skeletons[0].joints[j].z, 1e-5f));
    }
    /* a moving body never strays more than max_deviation_radius from the raw joints */
    for (i = 0; i < 120; i++) {
        l2k_synth_body(i / 30.0, 0, 1, &body);
        l2k_mapper_map(&m, &body, 1, up, 0, 0, L2K_SOURCE_SYNTHETIC, &f);
        raw = f;
        l2k_smoother_apply(&sm, &f, &p);
        for (j = 0; j < L2K_JOINT_COUNT; j++) {
            float dx = f.skeletons[0].joints[j].x - raw.skeletons[0].joints[j].x;
            float dy = f.skeletons[0].joints[j].y - raw.skeletons[0].joints[j].y;
            float dz = f.skeletons[0].joints[j].z - raw.skeletons[0].joints[j].z;
            float d = sqrtf(dx * dx + dy * dy + dz * dz);
            if (d > worst) worst = d;
        }
    }
    CHECK(worst <= p.max_deviation_radius + 1e-4f);
    CHECK(worst > 0.001f); /* it does filter */
}

static void test_cabinet_scene(void) {
    /* the synthetic figure seen like Dance Evolution's cabinet Kinect: 2 m up, tilted down */
    l2k_mapper m;
    l2k_src_body a, b;
    l2k_skeleton_frame f;
    l2k_vec3 up = l2k_synth_up(L2K_SYNTH_CABINET_TILT), level = l2k_synth_up(0.0f);
    static uint16_t img[320 * 240];
    const l2k_skeleton *s;
    float u, v, top = 240, bottom = 0;
    int i, j, n = 0;

    /* the level scene is unchanged: l2k_synth_body == l2k_synth_body_at(height 1 m, tilt 0) */
    l2k_synth_body(0.37, 1, 2, &a);
    l2k_synth_body_at(0.37, 1, 2, L2K_SYNTH_CAMERA_HEIGHT, 0.0f, &b);
    CHECK(memcmp(&a, &b, sizeof a) == 0);
    CHECK(level.x == L2K_SYNTH_UP_X && level.y == L2K_SYNTH_UP_Y && level.z == L2K_SYNTH_UP_Z);

    l2k_mapper_init(&m, NULL);
    for (i = 0; i < 30; i++) {
        l2k_synth_body_at(i / 30.0, 0, 1, L2K_SYNTH_CABINET_HEIGHT, L2K_SYNTH_CABINET_TILT, &a);
        l2k_mapper_map(&m, &a, 1, up, l2k_quadrant_from_up(up.x, up.y), (uint64_t)i * 33333, L2K_SOURCE_SYNTHETIC, &f);
    }
    s = &f.skeletons[0];
    CHECK(s->state == L2K_SKELETON_TRACKED);
    /* the sensor reports how it is placed: tilt -24 degrees, floor 2 m below along up */
    CHECK(nearly(l2k_elevation_degrees(f.up), L2K_SYNTH_CABINET_TILT, 0.01f));
    CHECK(nearly(f.up.y, 0.91355f, 1e-4f) && nearly(f.up.z, -0.40674f, 1e-4f));
    CHECK((f.flags & L2K_FRAME_FLOOR_VALID) && nearly(f.floor_plane.w, L2K_SYNTH_CABINET_HEIGHT, 0.06f));
    CHECK(nearly(f.floor_plane.z, f.up.z, 1e-6f));
    /* still upright along up: head above the hip centre by about 0.7 m */
    {
        l2k_vec3 h = s->joints[L2K_JOINT_HEAD], c = s->joints[L2K_JOINT_HIP_CENTER];
        float rise = f.up.x * (h.x - c.x) + f.up.y * (h.y - c.y) + f.up.z * (h.z - c.z);
        CHECK(rise > 0.6f && rise < 0.8f);
    }
    /* framed: every joint inside the 320x240 image, the figure spanning most of its height */
    for (j = 0; j < L2K_JOINT_COUNT; j++) {
        CHECK(l2k_skeleton_to_depth_pixel(s->joints[j], 320, 240, &u, &v) == 1);
        CHECK(u >= 0 && u < 320 && v >= 0 && v < 240);
        if (v < top) top = v;
        if (v > bottom) bottom = v;
    }
    CHECK(top > 20 && bottom < 225 && bottom - top > 120);
    /* the drawn silhouette is in the picture too */
    l2k_depth_rasterize(&f, img, 320, 240, 0);
    for (i = 0; i < 320 * 240; i++) n += l2k_depth_player(img[i]) == 1;
    CHECK(n > 1500);
}

int main(int argc, char **argv) {
    int i;
    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--write") == 0) g_write = 1;
        else g_dir = argv[i];
    }
    test_control_packets();
    test_skeleton_packet();
    test_depth_codec();
    test_chunks();
    test_mapping();
    test_upper_body();
    test_virtual_camera();
    test_depth_images();
    test_smoothing();
    test_cabinet_scene();
    if (g_write) printf("golden packets written to %s\n", g_dir);
    printf("%d checks, %d failed\n", g_checks, g_failed);
    return g_failed ? 1 : 0;
}
