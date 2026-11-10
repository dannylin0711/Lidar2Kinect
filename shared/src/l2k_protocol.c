/* Lidar2Kinect wire protocol "L2K1" - reference encoder/decoder. See docs/02-wire-protocol.md. */
#include "l2k/l2k_protocol.h"

#include <string.h>

/* ---- little-endian cursor ---- */

typedef struct {
    uint8_t *p;
    uint8_t *end;
    int ok;
} wr;

typedef struct {
    const uint8_t *p;
    const uint8_t *end;
    int ok;
} rd;

static void w_bytes(wr *w, const void *src, size_t n) {
    if (!w->ok || (size_t)(w->end - w->p) < n) {
        w->ok = 0;
        return;
    }
    if (n) memcpy(w->p, src, n);
    w->p += n;
}
static void w_u8(wr *w, uint8_t v) { w_bytes(w, &v, 1); }
static void w_u16(wr *w, uint16_t v) {
    uint8_t b[2] = {(uint8_t)v, (uint8_t)(v >> 8)};
    w_bytes(w, b, 2);
}
static void w_u32(wr *w, uint32_t v) {
    uint8_t b[4] = {(uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};
    w_bytes(w, b, 4);
}
static void w_u64(wr *w, uint64_t v) {
    w_u32(w, (uint32_t)v);
    w_u32(w, (uint32_t)(v >> 32));
}
static void w_f32(wr *w, float f) {
    uint32_t v;
    memcpy(&v, &f, 4);
    w_u32(w, v);
}
static void w_name(wr *w, const char *name) {
    uint8_t b[L2K_NAME_SIZE];
    size_t n = 0;
    memset(b, 0, sizeof b);
    while (n < L2K_NAME_SIZE && name[n]) {
        b[n] = (uint8_t)name[n];
        n++;
    }
    w_bytes(w, b, sizeof b);
}

static void r_bytes(rd *r, void *dst, size_t n) {
    if (!r->ok || (size_t)(r->end - r->p) < n) {
        r->ok = 0;
        memset(dst, 0, n);
        return;
    }
    memcpy(dst, r->p, n);
    r->p += n;
}
static uint8_t r_u8(rd *r) {
    uint8_t v;
    r_bytes(r, &v, 1);
    return v;
}
static uint16_t r_u16(rd *r) {
    uint8_t b[2];
    r_bytes(r, b, 2);
    return (uint16_t)(b[0] | (b[1] << 8));
}
static uint32_t r_u32(rd *r) {
    uint8_t b[4];
    r_bytes(r, b, 4);
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}
static uint64_t r_u64(rd *r) {
    uint64_t lo = r_u32(r);
    uint64_t hi = r_u32(r);
    return lo | (hi << 32);
}
static float r_f32(rd *r) {
    uint32_t v = r_u32(r);
    float f;
    memcpy(&f, &v, 4);
    return f;
}
static void r_name(rd *r, char *name) {
    r_bytes(r, name, L2K_NAME_SIZE);
    name[L2K_NAME_SIZE] = 0;
}

/* ---- header ---- */

static void begin(wr *w, uint8_t *buf, size_t cap, uint8_t type, uint32_t session, uint32_t seq) {
    w->p = buf;
    w->end = buf + cap;
    w->ok = buf != NULL;
    w_u32(w, L2K_MAGIC);
    w_u8(w, L2K_VERSION);
    w_u8(w, type);
    w_u16(w, 0); /* length, patched in finish() */
    w_u32(w, session);
    w_u32(w, seq);
}

static size_t finish(wr *w, uint8_t *buf) {
    size_t len;
    if (!w->ok) return 0;
    len = (size_t)(w->p - buf);
    if (len > 0xFFFF) return 0;
    buf[6] = (uint8_t)len;
    buf[7] = (uint8_t)(len >> 8);
    return len;
}

int l2k_decode_header(const uint8_t *buf, size_t len, l2k_header *out) {
    rd r;
    if (!buf || len < L2K_HEADER_SIZE) return L2K_ERR_SHORT;
    r.p = buf;
    r.end = buf + len;
    r.ok = 1;
    if (r_u32(&r) != L2K_MAGIC) return L2K_ERR_MAGIC;
    out->version = r_u8(&r);
    out->type = r_u8(&r);
    out->length = r_u16(&r);
    out->session = r_u32(&r);
    out->seq = r_u32(&r);
    if (out->version != L2K_VERSION) return L2K_ERR_VERSION;
    if (out->length < L2K_HEADER_SIZE || out->length > len) return L2K_ERR_SHORT;
    return L2K_OK;
}

/* Check the header and open a reader on the payload. */
static int open_payload(const uint8_t *buf, size_t len, uint8_t type, uint8_t type2, rd *r) {
    l2k_header h;
    int e = l2k_decode_header(buf, len, &h);
    if (e != L2K_OK) return e;
    if (h.type != type && h.type != type2) return L2K_ERR_TYPE;
    r->p = buf + L2K_HEADER_SIZE;
    r->end = buf + h.length;
    r->ok = 1;
    return L2K_OK;
}

/* ---- HELLO / CONFIG / BYE ---- */

size_t l2k_encode_hello(uint8_t *buf, size_t cap, uint32_t session, uint32_t seq, const l2k_hello *hello) {
    wr w;
    begin(&w, buf, cap, L2K_PKT_HELLO, session, seq);
    w_u64(&w, hello->sender_time_us);
    w_u32(&w, hello->caps);
    w_u8(&w, hello->max_tracked);
    w_u8(&w, hello->source_kind);
    w_u16(&w, 0);
    w_name(&w, hello->name);
    return finish(&w, buf);
}

int l2k_decode_hello(const uint8_t *buf, size_t len, l2k_hello *out) {
    rd r;
    int e = open_payload(buf, len, L2K_PKT_HELLO, L2K_PKT_HELLO, &r);
    if (e != L2K_OK) return e;
    out->sender_time_us = r_u64(&r);
    out->caps = r_u32(&r);
    out->max_tracked = r_u8(&r);
    out->source_kind = r_u8(&r);
    (void)r_u16(&r);
    r_name(&r, out->name);
    return r.ok ? L2K_OK : L2K_ERR_SHORT;
}

size_t l2k_encode_config(uint8_t *buf, size_t cap, uint32_t session, uint32_t seq, const l2k_config *config) {
    wr w;
    begin(&w, buf, cap, L2K_PKT_CONFIG, session, seq);
    w_u64(&w, config->echo_sender_time_us);
    w_u64(&w, config->receiver_time_us);
    w_u32(&w, config->streams);
    w_u32(&w, config->skeleton_flags);
    w_u8(&w, config->consumer_kind);
    w_u8(&w, config->depth_mode);
    w_u8(&w, config->color_quality);
    w_u8(&w, 0);
    w_name(&w, config->name);
    return finish(&w, buf);
}

int l2k_decode_config(const uint8_t *buf, size_t len, l2k_config *out) {
    rd r;
    int e = open_payload(buf, len, L2K_PKT_CONFIG, L2K_PKT_CONFIG, &r);
    if (e != L2K_OK) return e;
    out->echo_sender_time_us = r_u64(&r);
    out->receiver_time_us = r_u64(&r);
    out->streams = r_u32(&r);
    out->skeleton_flags = r_u32(&r);
    out->consumer_kind = r_u8(&r);
    out->depth_mode = r_u8(&r);
    out->color_quality = r_u8(&r);
    out->reserved = r_u8(&r);
    r_name(&r, out->name);
    return r.ok ? L2K_OK : L2K_ERR_SHORT;
}

size_t l2k_encode_bye(uint8_t *buf, size_t cap, uint32_t session, uint32_t seq) {
    wr w;
    begin(&w, buf, cap, L2K_PKT_BYE, session, seq);
    return finish(&w, buf);
}

/* ---- SKELETON ---- */

size_t l2k_encode_skeleton(uint8_t *buf, size_t cap, uint32_t session, uint32_t seq, const l2k_skeleton_frame *f) {
    wr w;
    int i, j;
    uint8_t count = 0;
    for (i = 0; i < L2K_SKELETON_SLOTS; i++) {
        if (f->skeletons[i].state > L2K_SKELETON_TRACKED) return 0;
        if (f->skeletons[i].state != L2K_SKELETON_NOT_TRACKED) count++;
    }
    begin(&w, buf, cap, L2K_PKT_SKELETON, session, seq);
    w_u64(&w, f->timestamp_us);
    w_u32(&w, f->frame_number);
    w_u32(&w, f->flags);
    w_f32(&w, f->floor_plane.x);
    w_f32(&w, f->floor_plane.y);
    w_f32(&w, f->floor_plane.z);
    w_f32(&w, f->floor_plane.w);
    w_f32(&w, f->up.x);
    w_f32(&w, f->up.y);
    w_f32(&w, f->up.z);
    w_u8(&w, count);
    w_u8(&w, f->source_kind);
    w_u16(&w, 0);
    for (i = 0; i < L2K_SKELETON_SLOTS; i++) {
        const l2k_skeleton *s = &f->skeletons[i];
        if (s->state == L2K_SKELETON_NOT_TRACKED) continue;
        w_u8(&w, (uint8_t)i);
        w_u8(&w, s->state);
        w_u16(&w, s->quality);
        w_u32(&w, s->tracking_id);
        w_f32(&w, s->position.x);
        w_f32(&w, s->position.y);
        w_f32(&w, s->position.z);
        if (s->state == L2K_SKELETON_TRACKED) {
            for (j = 0; j < L2K_JOINT_COUNT; j++) {
                w_f32(&w, s->joints[j].x);
                w_f32(&w, s->joints[j].y);
                w_f32(&w, s->joints[j].z);
            }
            w_bytes(&w, s->joint_state, L2K_JOINT_COUNT);
        }
    }
    return finish(&w, buf);
}

int l2k_decode_skeleton(const uint8_t *buf, size_t len, l2k_skeleton_frame *out) {
    rd r;
    int i, j, count;
    int e = open_payload(buf, len, L2K_PKT_SKELETON, L2K_PKT_SKELETON, &r);
    if (e != L2K_OK) return e;
    memset(out, 0, sizeof *out);
    out->timestamp_us = r_u64(&r);
    out->frame_number = r_u32(&r);
    out->flags = r_u32(&r);
    out->floor_plane.x = r_f32(&r);
    out->floor_plane.y = r_f32(&r);
    out->floor_plane.z = r_f32(&r);
    out->floor_plane.w = r_f32(&r);
    out->up.x = r_f32(&r);
    out->up.y = r_f32(&r);
    out->up.z = r_f32(&r);
    count = r_u8(&r);
    out->source_kind = r_u8(&r);
    (void)r_u16(&r);
    if (!r.ok) return L2K_ERR_SHORT;
    if (count > L2K_SKELETON_SLOTS) return L2K_ERR_FORMAT;
    for (i = 0; i < count; i++) {
        uint8_t slot = r_u8(&r);
        uint8_t state = r_u8(&r);
        l2k_skeleton *s;
        if (!r.ok) return L2K_ERR_SHORT;
        if (slot >= L2K_SKELETON_SLOTS) return L2K_ERR_FORMAT;
        if (state != L2K_SKELETON_POSITION_ONLY && state != L2K_SKELETON_TRACKED) return L2K_ERR_FORMAT;
        s = &out->skeletons[slot];
        if (s->state != L2K_SKELETON_NOT_TRACKED) return L2K_ERR_FORMAT; /* slot listed twice */
        s->state = state;
        s->quality = r_u16(&r);
        s->tracking_id = r_u32(&r);
        s->position.x = r_f32(&r);
        s->position.y = r_f32(&r);
        s->position.z = r_f32(&r);
        if (state == L2K_SKELETON_TRACKED) {
            for (j = 0; j < L2K_JOINT_COUNT; j++) {
                s->joints[j].x = r_f32(&r);
                s->joints[j].y = r_f32(&r);
                s->joints[j].z = r_f32(&r);
            }
            r_bytes(&r, s->joint_state, L2K_JOINT_COUNT);
            for (j = 0; j < L2K_JOINT_COUNT; j++) {
                if (s->joint_state[j] > L2K_JOINT_TRACKED) return L2K_ERR_FORMAT;
            }
        }
        if (!r.ok) return L2K_ERR_SHORT;
    }
    return L2K_OK;
}

/* ---- image chunks ---- */

uint32_t l2k_chunk_count(size_t total_len) {
    size_t n;
    if (total_len == 0) return 0;
    n = (total_len + L2K_CHUNK_PAYLOAD - 1) / L2K_CHUNK_PAYLOAD;
    return n > L2K_MAX_CHUNKS ? 0 : (uint32_t)n;
}

int l2k_chunk_slice(l2k_chunk *chunk, const uint8_t *data, size_t total_len, uint32_t index) {
    uint32_t count = l2k_chunk_count(total_len);
    size_t off, n;
    if (count == 0 || index >= count) return 0;
    off = (size_t)index * L2K_CHUNK_PAYLOAD;
    n = total_len - off;
    if (n > L2K_CHUNK_PAYLOAD) n = L2K_CHUNK_PAYLOAD;
    chunk->chunk_index = (uint16_t)index;
    chunk->chunk_count = (uint16_t)count;
    chunk->chunk_len = (uint16_t)n;
    chunk->total_len = (uint32_t)total_len;
    chunk->offset = (uint32_t)off;
    chunk->payload = data + off;
    return 1;
}

size_t l2k_encode_chunk(uint8_t *buf, size_t cap, uint32_t session, uint32_t seq, uint8_t type, const l2k_chunk *c) {
    wr w;
    if (type != L2K_PKT_DEPTH && type != L2K_PKT_COLOR) return 0;
    if (c->chunk_len > L2K_CHUNK_PAYLOAD || (c->chunk_len && !c->payload)) return 0;
    begin(&w, buf, cap, type, session, seq);
    w_u64(&w, c->timestamp_us);
    w_u32(&w, c->frame_number);
    w_u16(&w, c->width);
    w_u16(&w, c->height);
    w_u8(&w, c->encoding);
    w_u8(&w, c->flags);
    w_u16(&w, c->chunk_index);
    w_u16(&w, c->chunk_count);
    w_u16(&w, c->chunk_len);
    w_u32(&w, c->total_len);
    w_u32(&w, c->offset);
    w_bytes(&w, c->payload, c->chunk_len);
    return finish(&w, buf);
}

int l2k_decode_chunk(const uint8_t *buf, size_t len, l2k_chunk *out) {
    rd r;
    int e = open_payload(buf, len, L2K_PKT_DEPTH, L2K_PKT_COLOR, &r);
    if (e != L2K_OK) return e;
    out->timestamp_us = r_u64(&r);
    out->frame_number = r_u32(&r);
    out->width = r_u16(&r);
    out->height = r_u16(&r);
    out->encoding = r_u8(&r);
    out->flags = r_u8(&r);
    out->chunk_index = r_u16(&r);
    out->chunk_count = r_u16(&r);
    out->chunk_len = r_u16(&r);
    out->total_len = r_u32(&r);
    out->offset = r_u32(&r);
    if (!r.ok) return L2K_ERR_SHORT;
    if ((size_t)(r.end - r.p) < out->chunk_len) return L2K_ERR_SHORT;
    out->payload = r.p;
    /* consistency: fixed-size chunks, the last one may be short */
    if (out->chunk_count == 0 || out->chunk_count > L2K_MAX_CHUNKS) return L2K_ERR_FORMAT;
    if (out->chunk_index >= out->chunk_count) return L2K_ERR_FORMAT;
    if (out->chunk_len == 0 || out->chunk_len > L2K_CHUNK_PAYLOAD) return L2K_ERR_FORMAT;
    if (l2k_chunk_count(out->total_len) != out->chunk_count) return L2K_ERR_FORMAT;
    if (out->offset != (uint32_t)out->chunk_index * L2K_CHUNK_PAYLOAD) return L2K_ERR_FORMAT;
    if ((uint64_t)out->offset + out->chunk_len > out->total_len) return L2K_ERR_FORMAT;
    if (out->chunk_index + 1 < out->chunk_count && out->chunk_len != L2K_CHUNK_PAYLOAD) return L2K_ERR_FORMAT;
    if (out->chunk_index + 1 == out->chunk_count && out->offset + out->chunk_len != out->total_len) return L2K_ERR_FORMAT;
    return L2K_OK;
}

/* ---- reassembly ---- */

void l2k_assembler_init(l2k_assembler *a, uint8_t *storage, size_t capacity) {
    memset(a, 0, sizeof *a);
    a->data = storage;
    a->capacity = capacity;
}

int l2k_assembler_push(l2k_assembler *a, const l2k_chunk *c) {
    uint32_t idx = c->chunk_index;
    if (c->total_len > a->capacity) return L2K_ERR_SHORT;
    if (idx >= L2K_MAX_CHUNKS || idx >= c->chunk_count || !c->payload) return L2K_ERR_FORMAT;
    if ((uint64_t)c->offset + c->chunk_len > c->total_len) return L2K_ERR_FORMAT;
    if (a->active || a->complete) {
        if (c->frame_number != a->info.frame_number) {
            if (!l2k_seq_newer(c->frame_number, a->info.frame_number)) return L2K_ERR_STALE;
            if (a->active) a->frames_dropped++;
            a->active = 0;
            a->complete = 0;
        } else if (a->complete) {
            return 0; /* duplicate chunk of the frame already delivered */
        } else if (c->total_len != a->info.total_len || c->chunk_count != a->info.chunk_count) {
            return L2K_ERR_FORMAT;
        }
    }
    if (!a->active) {
        a->info = *c;
        a->info.payload = NULL;
        a->received = 0;
        memset(a->have, 0, sizeof a->have);
        a->active = 1;
    }
    if (a->have[idx >> 3] & (1u << (idx & 7))) return 0;
    memcpy(a->data + c->offset, c->payload, c->chunk_len);
    a->have[idx >> 3] |= (uint8_t)(1u << (idx & 7));
    a->received++;
    if (a->received == a->info.chunk_count) {
        a->active = 0;
        a->complete = 1;
        a->frames_completed++;
        return 1;
    }
    return 0;
}
