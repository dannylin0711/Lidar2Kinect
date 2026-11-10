/* Lidar2Kinect - virtual Kinect camera geometry, depth image construction, depth codec D1. */
#include "l2k/l2k_depth.h"

#include <math.h>
#include <string.h>

/* ---- geometry ---- */

int l2k_quadrant_from_up(float up_x, float up_y) {
    if (fabsf(up_y) >= fabsf(up_x)) return up_y >= 0.0f ? 0 : 2;
    return up_x > 0.0f ? 1 : 3;
}

l2k_vec3 l2k_camera_to_kinect(l2k_vec3 p, int quadrant) {
    l2k_vec3 k;
    switch (quadrant & 3) {
    default:
    case 0: k.x = -p.x; k.y = p.y; break;
    case 1: k.x = p.y; k.y = p.x; break;
    case 2: k.x = p.x; k.y = -p.y; break;
    case 3: k.x = -p.y; k.y = -p.x; break;
    }
    k.z = -p.z;
    return k;
}

void l2k_virtual_to_source(const l2k_src_camera *cam, int quadrant, float un, float vn, float *sx, float *sy) {
    /* ray of the virtual pixel in skeleton space, per unit depth */
    float xk = (2.0f * un - 1.0f) * L2K_TAN_HALF_X;
    float yk = (1.0f - 2.0f * vn) * L2K_TAN_HALF_Y;
    float xc, yc; /* the same ray in source camera space (x right, y up), per unit forward depth */
    switch (quadrant & 3) {
    default:
    case 0: xc = -xk; yc = yk; break;
    case 1: xc = yk; yc = xk; break;
    case 2: xc = xk; yc = -yk; break;
    case 3: xc = -yk; yc = -xk; break;
    }
    *sx = cam->cxn + cam->fxn * xc;
    *sy = cam->cyn - cam->fyn * yc;
}

int l2k_skeleton_to_depth_pixel(l2k_vec3 p, int w, int h, float *u, float *v) {
    if (!(p.z > 0.0f)) return 0;
    *u = (float)w * (0.5f + p.x * (L2K_FOCAL_320 / p.z) / 320.0f);
    *v = (float)h * (0.5f - p.y * (L2K_FOCAL_320 / p.z) / 240.0f);
    return 1;
}

/* ---- resampling the source depth / mask ---- */

void l2k_depth_render(const l2k_depth_source *src, const l2k_src_camera *cam, int quadrant,
                      uint16_t *out, int out_w, int out_h) {
    int u, v;
    for (v = 0; v < out_h; v++) {
        for (u = 0; u < out_w; u++) {
            float sx, sy;
            uint32_t mm = 0, player = 0;
            l2k_virtual_to_source(cam, quadrant, ((float)u + 0.5f) / (float)out_w, ((float)v + 0.5f) / (float)out_h, &sx, &sy);
            if (sx >= 0.0f && sx < 1.0f && sy >= 0.0f && sy < 1.0f) {
                if (src->mask) {
                    int mx = (int)(sx * (float)src->mask_w), my = (int)(sy * (float)src->mask_h);
                    if (src->mask[(size_t)my * src->mask_stride + (size_t)mx] >= src->mask_threshold) player = src->player_index;
                }
                if (src->depth_m && (player || !src->players_only)) {
                    int dx = (int)(sx * (float)src->depth_w), dy = (int)(sy * (float)src->depth_h);
                    const float *row = (const float *)(const void *)((const uint8_t *)src->depth_m + (size_t)dy * src->depth_stride);
                    float d = row[dx];
                    if (d > 0.0f) { /* false for NaN */
                        float f = d * 1000.0f + 0.5f;
                        if (f >= (float)L2K_DEPTH_MIN_MM && f <= (float)L2K_DEPTH_MAX_MM) mm = (uint32_t)f;
                    }
                }
                if (player && mm == 0 && src->fallback_depth_m > 0.0f) mm = (uint32_t)(src->fallback_depth_m * 1000.0f + 0.5f);
            }
            out[(size_t)v * (size_t)out_w + (size_t)u] = l2k_depth_pack(mm, player);
        }
    }
}

/* ---- capsule figure ---- */

typedef struct {
    uint8_t a, b;
    float radius; /* metres */
} bone;

static const bone BONES[] = {
    {L2K_JOINT_HEAD, L2K_JOINT_HEAD, 0.11f},
    {L2K_JOINT_SHOULDER_CENTER, L2K_JOINT_HEAD, 0.06f},
    {L2K_JOINT_SHOULDER_CENTER, L2K_JOINT_SPINE, 0.15f},
    {L2K_JOINT_SPINE, L2K_JOINT_HIP_CENTER, 0.15f},
    {L2K_JOINT_SHOULDER_LEFT, L2K_JOINT_SHOULDER_RIGHT, 0.08f},
    {L2K_JOINT_HIP_LEFT, L2K_JOINT_HIP_RIGHT, 0.10f},
    {L2K_JOINT_SHOULDER_LEFT, L2K_JOINT_ELBOW_LEFT, 0.05f},
    {L2K_JOINT_ELBOW_LEFT, L2K_JOINT_WRIST_LEFT, 0.04f},
    {L2K_JOINT_WRIST_LEFT, L2K_JOINT_HAND_LEFT, 0.05f},
    {L2K_JOINT_SHOULDER_RIGHT, L2K_JOINT_ELBOW_RIGHT, 0.05f},
    {L2K_JOINT_ELBOW_RIGHT, L2K_JOINT_WRIST_RIGHT, 0.04f},
    {L2K_JOINT_WRIST_RIGHT, L2K_JOINT_HAND_RIGHT, 0.05f},
    {L2K_JOINT_HIP_LEFT, L2K_JOINT_KNEE_LEFT, 0.08f},
    {L2K_JOINT_KNEE_LEFT, L2K_JOINT_ANKLE_LEFT, 0.06f},
    {L2K_JOINT_ANKLE_LEFT, L2K_JOINT_FOOT_LEFT, 0.05f},
    {L2K_JOINT_HIP_RIGHT, L2K_JOINT_KNEE_RIGHT, 0.08f},
    {L2K_JOINT_KNEE_RIGHT, L2K_JOINT_ANKLE_RIGHT, 0.06f},
    {L2K_JOINT_ANKLE_RIGHT, L2K_JOINT_FOOT_RIGHT, 0.05f},
};

static void draw_capsule(uint16_t *out, int w, int h, l2k_vec3 pa, l2k_vec3 pb, float radius, uint32_t player) {
    float ua, va, ub, vb, ra, rb, rmax, du, dv, len2;
    int x0, x1, y0, y1, x, y;
    float scale = L2K_FOCAL_320 * (float)w / 320.0f; /* focal length in pixels of this image */
    if (!l2k_skeleton_to_depth_pixel(pa, w, h, &ua, &va)) return;
    if (!l2k_skeleton_to_depth_pixel(pb, w, h, &ub, &vb)) return;
    ra = radius * scale / pa.z;
    rb = radius * scale / pb.z;
    rmax = ra > rb ? ra : rb;
    x0 = (int)floorf((ua < ub ? ua : ub) - rmax);
    x1 = (int)ceilf((ua > ub ? ua : ub) + rmax);
    y0 = (int)floorf((va < vb ? va : vb) - rmax);
    y1 = (int)ceilf((va > vb ? va : vb) + rmax);
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > w - 1) x1 = w - 1;
    if (y1 > h - 1) y1 = h - 1;
    du = ub - ua;
    dv = vb - va;
    len2 = du * du + dv * dv;
    for (y = y0; y <= y1; y++) {
        for (x = x0; x <= x1; x++) {
            float px = (float)x + 0.5f - ua, py = (float)y + 0.5f - va;
            float t = len2 > 0.0f ? (px * du + py * dv) / len2 : 0.0f;
            float ex, ey, r, z, f;
            uint16_t *dst;
            uint32_t mm;
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            ex = px - t * du;
            ey = py - t * dv;
            r = ra + (rb - ra) * t;
            if (ex * ex + ey * ey > r * r) continue;
            z = pa.z + (pb.z - pa.z) * t;
            f = z * 1000.0f + 0.5f;
            if (f < (float)L2K_DEPTH_MIN_MM || f > (float)L2K_DEPTH_MAX_MM) continue;
            mm = (uint32_t)f;
            dst = &out[(size_t)y * (size_t)w + (size_t)x];
            if (l2k_depth_player(*dst) != 0 && l2k_depth_mm(*dst) != 0 && l2k_depth_mm(*dst) <= mm) continue;
            *dst = l2k_depth_pack(mm, player);
        }
    }
}

void l2k_depth_rasterize(const l2k_skeleton_frame *frame, uint16_t *out, int w, int h, int keep) {
    int s;
    size_t i;
    if (!keep) memset(out, 0, (size_t)w * (size_t)h * sizeof(uint16_t));
    for (s = 0; s < L2K_SKELETON_SLOTS; s++) {
        const l2k_skeleton *sk = &frame->skeletons[s];
        if (sk->state != L2K_SKELETON_TRACKED) continue;
        for (i = 0; i < sizeof BONES / sizeof BONES[0]; i++) {
            const bone *b = &BONES[i];
            if (sk->joint_state[b->a] == L2K_JOINT_NOT_TRACKED || sk->joint_state[b->b] == L2K_JOINT_NOT_TRACKED) continue;
            draw_capsule(out, w, h, sk->joints[b->a], sk->joints[b->b], b->radius, (uint32_t)s + 1);
        }
    }
}

/* ---- codec D1 ---- */

size_t l2k_depth_d1_bound(size_t pixel_count) {
    return pixel_count * 3;
}

size_t l2k_depth_d1_encode(const uint16_t *px, size_t count, uint8_t *out, size_t cap) {
    size_t i = 0, o = 0;
    uint16_t prev = 0;
    while (i < count) {
        uint16_t v = px[i];
        if (v == prev) {
            size_t run = 1;
            while (run < 128 && i + run < count && px[i + run] == prev) run++;
            if (o + 1 > cap) return 0;
            out[o++] = (uint8_t)(run - 1);
            i += run;
            continue;
        }
        if (((v ^ prev) & L2K_PLAYER_INDEX_MASK) == 0) {
            int d = (int)(v >> L2K_PLAYER_INDEX_BITS) - (int)(prev >> L2K_PLAYER_INDEX_BITS);
            if (d >= -63 && d <= 63) {
                if (o + 1 > cap) return 0;
                out[o++] = (uint8_t)(0xBF + d);
                prev = v;
                i++;
                continue;
            }
        }
        if (o + 3 > cap) return 0;
        out[o++] = 0xFF;
        out[o++] = (uint8_t)v;
        out[o++] = (uint8_t)(v >> 8);
        prev = v;
        i++;
    }
    return o;
}

int l2k_depth_d1_decode(const uint8_t *in, size_t len, uint16_t *px, size_t count) {
    size_t i = 0, o = 0;
    uint16_t prev = 0;
    while (i < len) {
        uint8_t op = in[i++];
        if (op < 0x80) {
            size_t run = (size_t)op + 1;
            if (o + run > count) return -1;
            while (run--) px[o++] = prev;
        } else if (op != 0xFF) {
            int d = (int)op - 0xBF;
            int depth = (int)(prev >> L2K_PLAYER_INDEX_BITS) + d;
            if (d == 0 || depth < 0 || depth > 0x1FFF || o >= count) return -1;
            prev = (uint16_t)((depth << L2K_PLAYER_INDEX_BITS) | (prev & L2K_PLAYER_INDEX_MASK));
            px[o++] = prev;
        } else {
            if (i + 2 > len || o >= count) return -1;
            prev = (uint16_t)(in[i] | (in[i + 1] << 8));
            i += 2;
            px[o++] = prev;
        }
    }
    return o == count ? 0 : -1;
}
