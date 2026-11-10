/* Lidar2Kinect - body source -> Kinect skeleton frame, and joint smoothing. */
#include "l2k/l2k_map.h"
#include "l2k/l2k_depth.h"

#include <math.h>
#include <string.h>

static l2k_vec3 v_add(l2k_vec3 a, l2k_vec3 b) { l2k_vec3 r = {a.x + b.x, a.y + b.y, a.z + b.z}; return r; }
static l2k_vec3 v_sub(l2k_vec3 a, l2k_vec3 b) { l2k_vec3 r = {a.x - b.x, a.y - b.y, a.z - b.z}; return r; }
static l2k_vec3 v_scale(l2k_vec3 a, float s) { l2k_vec3 r = {a.x * s, a.y * s, a.z * s}; return r; }
static float v_dot(l2k_vec3 a, l2k_vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static l2k_vec3 v_cross(l2k_vec3 a, l2k_vec3 b) {
    l2k_vec3 r = {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    return r;
}
static float v_len(l2k_vec3 a) { return sqrtf(v_dot(a, a)); }
static l2k_vec3 v_unit(l2k_vec3 a, l2k_vec3 fallback) {
    float l = v_len(a);
    return l > 1e-6f ? v_scale(a, 1.0f / l) : fallback;
}
static uint8_t u8_min(uint8_t a, uint8_t b) { return a < b ? a : b; }

void l2k_map_params_default(l2k_map_params *p) {
    p->hip_center_lift = 0.10f;
    p->spine_fraction = 0.30f;
    p->hand_extension = 0.30f;
    p->foot_length = 0.13f;
    p->foot_drop = 0.06f;
    p->ankle_height = 0.08f;
    p->toe_height = 0.03f;
    p->clip_margin = 0.97f;
}

void l2k_mapper_init(l2k_mapper *m, const l2k_map_params *params) {
    memset(m, 0, sizeof *m);
    if (params) m->params = *params;
    else l2k_map_params_default(&m->params);
}

void l2k_mapper_set_floor(l2k_mapper *m, float sensor_height_m) {
    if (sensor_height_m > 0.0f) {
        m->sensor_height = sensor_height_m;
        m->floor_valid = 1;
        m->floor_fixed = 1;
    } else {
        m->floor_fixed = 0;
    }
}

float l2k_elevation_degrees(l2k_vec3 up) {
    /* The optical axis is +z. A sensor looking upwards by e sees world up as (0, cos e, sin e). */
    float s = up.z;
    if (s > 1.0f) s = 1.0f;
    if (s < -1.0f) s = -1.0f;
    return asinf(s) * 57.29578f;
}

/* Kinect joint -> parent joint (a missing joint takes its parent's position). */
static const uint8_t PARENT[L2K_JOINT_COUNT] = {0, 0, 1, 2, 2, 4, 5, 6, 2, 8, 9, 10, 0, 12, 13, 14, 0, 16, 17, 18};

/* Kinect joints copied straight from a source joint. */
static const struct {
    uint8_t joint, src;
} DIRECT[] = {
    {L2K_JOINT_SHOULDER_CENTER, L2K_SRC_NECK},
    {L2K_JOINT_HEAD, L2K_SRC_HEAD},
    {L2K_JOINT_SHOULDER_LEFT, L2K_SRC_SHOULDER_LEFT},
    {L2K_JOINT_ELBOW_LEFT, L2K_SRC_ELBOW_LEFT},
    {L2K_JOINT_WRIST_LEFT, L2K_SRC_WRIST_LEFT},
    {L2K_JOINT_SHOULDER_RIGHT, L2K_SRC_SHOULDER_RIGHT},
    {L2K_JOINT_ELBOW_RIGHT, L2K_SRC_ELBOW_RIGHT},
    {L2K_JOINT_WRIST_RIGHT, L2K_SRC_WRIST_RIGHT},
    {L2K_JOINT_HIP_LEFT, L2K_SRC_HIP_LEFT},
    {L2K_JOINT_KNEE_LEFT, L2K_SRC_KNEE_LEFT},
    {L2K_JOINT_ANKLE_LEFT, L2K_SRC_ANKLE_LEFT},
    {L2K_JOINT_HIP_RIGHT, L2K_SRC_HIP_RIGHT},
    {L2K_JOINT_KNEE_RIGHT, L2K_SRC_KNEE_RIGHT},
    {L2K_JOINT_ANKLE_RIGHT, L2K_SRC_ANKLE_RIGHT},
};

/* Fill one skeleton from one source body. Returns the resulting slot state. */
static uint8_t map_body(const l2k_map_params *prm, const l2k_src_body *b, int quadrant, l2k_vec3 up, l2k_skeleton *s) {
    l2k_vec3 k[L2K_SRC_JOINT_COUNT];
    uint8_t conf[L2K_SRC_JOINT_COUNT];
    l2k_vec3 pelvis, neck, torso, sum = {0, 0, 0};
    uint8_t pelvis_conf, core_conf;
    int i, known = 0;
    size_t d;

    for (i = 0; i < L2K_SRC_JOINT_COUNT; i++) {
        conf[i] = b->conf[i] > 2 ? 2 : b->conf[i];
        k[i] = l2k_camera_to_kinect(b->p[i], quadrant);
        if (conf[i]) {
            sum = v_add(sum, k[i]);
            known++;
        }
    }
    if (!known) return L2K_SKELETON_NOT_TRACKED;

    pelvis = k[L2K_SRC_PELVIS];
    pelvis_conf = conf[L2K_SRC_PELVIS];
    if (!pelvis_conf && conf[L2K_SRC_HIP_LEFT] && conf[L2K_SRC_HIP_RIGHT]) {
        pelvis = v_scale(v_add(k[L2K_SRC_HIP_LEFT], k[L2K_SRC_HIP_RIGHT]), 0.5f);
        pelvis_conf = u8_min(conf[L2K_SRC_HIP_LEFT], conf[L2K_SRC_HIP_RIGHT]);
    }
    if (!pelvis_conf || !conf[L2K_SRC_NECK]) {
        /* not enough for a skeleton: report where the body is */
        s->position = v_scale(sum, 1.0f / (float)known);
        return L2K_SKELETON_POSITION_ONLY;
    }
    neck = k[L2K_SRC_NECK];
    torso = v_sub(neck, pelvis);
    core_conf = u8_min(pelvis_conf, conf[L2K_SRC_NECK]);

    memset(s->joint_state, L2K_JOINT_NOT_TRACKED, sizeof s->joint_state);
    s->joints[L2K_JOINT_HIP_CENTER] = v_add(pelvis, v_scale(torso, prm->hip_center_lift));
    s->joint_state[L2K_JOINT_HIP_CENTER] = core_conf;
    s->joints[L2K_JOINT_SPINE] = v_add(pelvis, v_scale(torso, prm->spine_fraction));
    s->joint_state[L2K_JOINT_SPINE] = core_conf;
    for (d = 0; d < sizeof DIRECT / sizeof DIRECT[0]; d++) {
        s->joints[DIRECT[d].joint] = k[DIRECT[d].src];
        s->joint_state[DIRECT[d].joint] = conf[DIRECT[d].src];
    }

    /* hands: the source's palm joint, else the forearm extended past the wrist */
    for (i = 0; i < 2; i++) {
        int hand = i ? L2K_SRC_HAND_RIGHT : L2K_SRC_HAND_LEFT;
        int wrist = i ? L2K_SRC_WRIST_RIGHT : L2K_SRC_WRIST_LEFT;
        int elbow = i ? L2K_SRC_ELBOW_RIGHT : L2K_SRC_ELBOW_LEFT;
        int out = i ? L2K_JOINT_HAND_RIGHT : L2K_JOINT_HAND_LEFT;
        if (conf[hand]) {
            s->joints[out] = k[hand];
            s->joint_state[out] = conf[hand];
        } else if (conf[wrist] && conf[elbow]) {
            s->joints[out] = v_add(k[wrist], v_scale(v_sub(k[wrist], k[elbow]), prm->hand_extension));
            s->joint_state[out] = L2K_JOINT_INFERRED;
        }
    }

    /* feet: the source's toe joint, else a point in front of the ankle */
    {
        l2k_vec3 fwd_default = {0, 0, -1};
        l2k_vec3 right = v_sub(k[L2K_SRC_HIP_RIGHT], k[L2K_SRC_HIP_LEFT]);
        l2k_vec3 facing = (conf[L2K_SRC_HIP_LEFT] && conf[L2K_SRC_HIP_RIGHT]) ? v_unit(v_cross(up, right), fwd_default) : fwd_default;
        for (i = 0; i < 2; i++) {
            int toe = i ? L2K_SRC_TOE_RIGHT : L2K_SRC_TOE_LEFT;
            int ankle = i ? L2K_SRC_ANKLE_RIGHT : L2K_SRC_ANKLE_LEFT;
            int out = i ? L2K_JOINT_FOOT_RIGHT : L2K_JOINT_FOOT_LEFT;
            if (conf[toe]) {
                s->joints[out] = k[toe];
                s->joint_state[out] = conf[toe];
            } else if (conf[ankle]) {
                s->joints[out] = v_add(k[ankle], v_sub(v_scale(facing, prm->foot_length), v_scale(up, prm->foot_drop)));
                s->joint_state[out] = L2K_JOINT_INFERRED;
            }
        }
    }

    /* a joint the source did not give sits on its parent, flagged not tracked */
    for (i = 1; i < L2K_JOINT_COUNT; i++) {
        if (s->joint_state[i] == L2K_JOINT_NOT_TRACKED) s->joints[i] = s->joints[PARENT[i]];
    }

    /* which image edges cut the body */
    s->quality = 0;
    for (i = 0; i < L2K_JOINT_COUNT; i++) {
        l2k_vec3 p = s->joints[i];
        float lx, ly;
        if (s->joint_state[i] == L2K_JOINT_NOT_TRACKED || !(p.z > 0.0f)) continue;
        lx = prm->clip_margin * L2K_TAN_HALF_X * p.z;
        ly = prm->clip_margin * L2K_TAN_HALF_Y * p.z;
        if (p.x > lx) s->quality |= L2K_QUALITY_CLIPPED_RIGHT;
        if (p.x < -lx) s->quality |= L2K_QUALITY_CLIPPED_LEFT;
        if (p.y > ly) s->quality |= L2K_QUALITY_CLIPPED_TOP;
        if (p.y < -ly) s->quality |= L2K_QUALITY_CLIPPED_BOTTOM;
    }

    s->position = s->joints[L2K_JOINT_HIP_CENTER];
    return L2K_SKELETON_TRACKED;
}

/* Height of the floor under one tracked skeleton, along `up` (negative: below the sensor). */
static int floor_under(const l2k_map_params *prm, const l2k_skeleton *s, l2k_vec3 up, float *h) {
    static const uint8_t FEET[4] = {L2K_JOINT_ANKLE_LEFT, L2K_JOINT_ANKLE_RIGHT, L2K_JOINT_FOOT_LEFT, L2K_JOINT_FOOT_RIGHT};
    int i, found = 0;
    for (i = 0; i < 4; i++) {
        float y;
        if (s->joint_state[FEET[i]] != L2K_JOINT_TRACKED) continue;
        y = v_dot(up, s->joints[FEET[i]]) - (i < 2 ? prm->ankle_height : prm->toe_height);
        if (!found || y < *h) *h = y;
        found = 1;
    }
    return found;
}

void l2k_mapper_map(l2k_mapper *m, const l2k_src_body *bodies, int body_count, l2k_vec3 up_camera,
                    int quadrant, uint64_t timestamp_us, uint8_t source_kind, l2k_skeleton_frame *out) {
    l2k_vec3 up_default = {0, 1, 0};
    l2k_vec3 up = v_unit(l2k_camera_to_kinect(up_camera, quadrant), up_default);
    uint32_t ids[L2K_SKELETON_SLOTS];
    int slot_of[L2K_SKELETON_SLOTS];
    int i, s, n = body_count > L2K_SKELETON_SLOTS ? L2K_SKELETON_SLOTS : body_count;
    int have_floor = 0;
    float floor_h = 0.0f;

    memset(out, 0, sizeof *out);
    out->timestamp_us = timestamp_us;
    out->frame_number = ++m->frame_number;
    out->source_kind = source_kind;
    out->up = up;

    /* slots: a body keeps its slot while its id stays; a vanished id frees its slot */
    for (i = 0; i < n; i++) {
        ids[i] = bodies[i].tracking_id ? bodies[i].tracking_id : 0x80000000u + (uint32_t)i;
        slot_of[i] = -1;
    }
    for (s = 0; s < L2K_SKELETON_SLOTS; s++) {
        int keep = 0;
        if (!m->slot_id[s]) continue;
        for (i = 0; i < n; i++) {
            if (ids[i] == m->slot_id[s] && slot_of[i] < 0) {
                slot_of[i] = s;
                keep = 1;
                break;
            }
        }
        if (!keep) m->slot_id[s] = 0;
    }
    for (i = 0; i < n; i++) {
        if (slot_of[i] >= 0) continue;
        for (s = 0; s < L2K_SKELETON_SLOTS; s++) {
            if (!m->slot_id[s]) {
                m->slot_id[s] = ids[i];
                slot_of[i] = s;
                break;
            }
        }
    }

    for (i = 0; i < n; i++) {
        l2k_skeleton *sk;
        float h;
        if (slot_of[i] < 0) continue;
        sk = &out->skeletons[slot_of[i]];
        sk->state = map_body(&m->params, &bodies[i], quadrant, up, sk);
        if (sk->state == L2K_SKELETON_NOT_TRACKED) {
            m->slot_id[slot_of[i]] = 0;
            continue;
        }
        sk->tracking_id = ids[i];
        if (sk->state == L2K_SKELETON_TRACKED && floor_under(&m->params, sk, up, &h)) {
            if (!have_floor || h < floor_h) floor_h = h;
            have_floor = 1;
        }
    }

    /* floor: follows a lower reading quickly and a higher one slowly, so a jump does not lift it */
    if (!m->floor_fixed && have_floor) {
        float height = -floor_h;
        if (height > 0.05f && height < 4.0f) {
            if (!m->floor_valid) m->sensor_height = height;
            else m->sensor_height += (height - m->sensor_height) * (height > m->sensor_height ? 0.5f : 0.01f);
            m->floor_valid = 1;
        }
    }
    if (m->floor_valid) {
        out->flags |= L2K_FRAME_FLOOR_VALID;
        out->floor_plane.x = up.x;
        out->floor_plane.y = up.y;
        out->floor_plane.z = up.z;
        out->floor_plane.w = m->sensor_height;
    }
}

/* ---- smoothing ---- */

void l2k_smooth_params_default(l2k_smooth_params *p) {
    p->smoothing = 0.5f;
    p->correction = 0.5f;
    p->prediction = 0.5f;
    p->jitter_radius = 0.05f;
    p->max_deviation_radius = 0.04f;
}

void l2k_smoother_init(l2k_smoother *s) {
    memset(s, 0, sizeof *s);
}

void l2k_smoother_apply(l2k_smoother *st, l2k_skeleton_frame *frame, const l2k_smooth_params *prm) {
    int s, j;
    for (s = 0; s < L2K_SKELETON_SLOTS; s++) {
        l2k_skeleton *sk = &frame->skeletons[s];
        if (sk->state != L2K_SKELETON_TRACKED) {
            memset(&st->slot[s], 0, sizeof st->slot[s]);
            continue;
        }
        if (st->slot[s].tracking_id != sk->tracking_id) {
            memset(&st->slot[s], 0, sizeof st->slot[s]);
            st->slot[s].tracking_id = sk->tracking_id;
        }
        for (j = 0; j < L2K_JOINT_COUNT; j++) {
            l2k_vec3 raw = sk->joints[j];
            l2k_vec3 prev_raw = st->slot[s].joint[j].raw;
            l2k_vec3 prev_filtered = st->slot[s].joint[j].filtered;
            l2k_vec3 prev_trend = st->slot[s].joint[j].trend;
            l2k_vec3 filtered, trend, predicted, diff;
            float len;
            uint32_t *count = &st->slot[s].joint[j].count;

            /* a joint at the origin is invalid: restart its filter */
            if (raw.x == 0.0f && raw.y == 0.0f && raw.z == 0.0f) *count = 0;

            if (*count == 0) {
                filtered = raw;
                trend.x = trend.y = trend.z = 0.0f;
                (*count)++;
            } else if (*count == 1) {
                filtered = v_scale(v_add(raw, prev_raw), 0.5f);
                diff = v_sub(filtered, prev_filtered);
                trend = v_add(v_scale(diff, prm->correction), v_scale(prev_trend, 1.0f - prm->correction));
                (*count)++;
            } else {
                /* jitter filter */
                diff = v_sub(raw, prev_filtered);
                len = v_len(diff);
                if (len <= prm->jitter_radius && prm->jitter_radius > 0.0f) {
                    float w = len / prm->jitter_radius;
                    filtered = v_add(v_scale(raw, w), v_scale(prev_filtered, 1.0f - w));
                } else {
                    filtered = raw;
                }
                /* double exponential smoothing */
                filtered = v_add(v_scale(filtered, 1.0f - prm->smoothing),
                                 v_scale(v_add(prev_filtered, prev_trend), prm->smoothing));
                diff = v_sub(filtered, prev_filtered);
                trend = v_add(v_scale(diff, prm->correction), v_scale(prev_trend, 1.0f - prm->correction));
            }

            /* predict ahead, but stay within max_deviation_radius of the raw position */
            predicted = v_add(filtered, v_scale(trend, prm->prediction));
            diff = v_sub(predicted, raw);
            len = v_len(diff);
            if (len > prm->max_deviation_radius && len > 0.0f) {
                float w = prm->max_deviation_radius / len;
                predicted = v_add(v_scale(predicted, w), v_scale(raw, 1.0f - w));
            }

            st->slot[s].joint[j].raw = raw;
            st->slot[s].joint[j].filtered = filtered;
            st->slot[s].joint[j].trend = trend;
            sk->joints[j] = predicted;
        }
    }
}
