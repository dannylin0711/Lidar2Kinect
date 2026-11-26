/* Lidar2Kinect - body source -> Kinect skeleton frame, and joint smoothing. */
#include "l2k/l2k_map.h"
#include "l2k/l2k_depth.h"
#include "l2k/l2k_protocol.h"

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
    p->upper_body = 0;
    p->upper_torso_ratio = 1.40f;
    p->upper_hip_ratio = 0.26f;
    p->upper_leg_ratio = 1.60f;
    p->hold_seconds = 0.5f;
    p->keep_identity = 1;
    p->identity_seconds = 0.0f;
    p->fit_play_area = 0;
    p->normalize = 0;
    p->norm_players = 1;
    p->norm_follow_seconds = 2.0f;
    l2k_map_params_set_game(p, L2K_GAME_DANCE_EVOLUTION); /* the play area and the standing point */
}

int l2k_game_of_consumer(uint8_t consumer_kind) {
    return consumer_kind == L2K_CONSUMER_VP4U ? L2K_GAME_DANCEAROUND : L2K_GAME_DANCE_EVOLUTION;
}

void l2k_map_params_set_game(l2k_map_params *p, int game) {
    if (game == L2K_GAME_DANCEAROUND) {
        /* SpineBase inside x 0.55 +- 0.9, z 2.30 +- 0.505 of the game's frame (udnappsetting.json's play area with its
         * margin, windows/docs/vp4u-interface.md section 5), and windows/vp4u puts the phone at x 0.55, z 0: so
         * 1.795 .. 2.805 m from the phone and 0.9 m either side. Stay 10 cm inside, as for Dance Evolution. */
        p->area_near = 1.90f;
        p->area_far = 2.70f;
        p->area_half_width = 0.80f;
        p->area_hip_min = 0.40f; /* the game takes a SpineBase 0.06 .. 2.26 m above the floor (camera 1.16 m +- 1.1) */
        p->area_hip_max = 1.60f;
        p->norm_distance = 2.30f; /* the play area's centre */
        p->norm_spacing = 0.61f;  /* unused: the game takes one player */
        p->norm_reach = 0.40f;    /* the play area reaches 0.505 m before and behind */
    } else {
        /* Dance Evolution (docs/01 section 8) */
        p->area_near = 1.90f;      /* the game accepts 1.80 .. 3.20 m: stay 10 cm inside */
        p->area_far = 3.10f;
        p->area_half_width = 1.15f; /* the game accepts +-1.25 m */
        p->area_hip_min = 0.40f;    /* the game accepts a hip 0 .. 2 m above its floor */
        p->area_hip_max = 1.60f;
        p->norm_distance = 2.5f;    /* the game's world origin: the middle of its play area */
        p->norm_spacing = 0.61f;    /* its two-player standing points */
        p->norm_reach = 0.5f;       /* the play area reaches 0.7 m before and behind, 1.25 m either side */
    }
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
typedef struct {
    l2k_vec3 offset[L2K_JOINT_COUNT];
    uint8_t valid[L2K_JOINT_COUNT];
} joint_memory;

/* The 30-joint body's extra joints (L2K_BODY_EXTRA_COUNT, in the order of l2k_mapper.extra), the body joint each is,
 * and the source joint that gives it (-1: none, always placed from the skeleton). */
static const uint8_t EXTRA_BODY[L2K_BODY_EXTRA_COUNT] = {
    L2K_BODY_NECK, L2K_BODY_HAND_TIP_LEFT, L2K_BODY_THUMB_LEFT, L2K_BODY_HAND_TIP_RIGHT, L2K_BODY_THUMB_RIGHT,
    L2K_BODY_NOSE, L2K_BODY_EYE_LEFT, L2K_BODY_EYE_RIGHT, L2K_BODY_EAR_LEFT, L2K_BODY_EAR_RIGHT};
static const int8_t EXTRA_SRC[L2K_BODY_EXTRA_COUNT] = {
    -1, L2K_SRC_HAND_TIP_LEFT, L2K_SRC_THUMB_LEFT, L2K_SRC_HAND_TIP_RIGHT, L2K_SRC_THUMB_RIGHT,
    L2K_SRC_NOSE, L2K_SRC_EYE_LEFT, L2K_SRC_EYE_RIGHT, L2K_SRC_EAR_LEFT, L2K_SRC_EAR_RIGHT};

/* Placement of the extra joints from a tracked skeleton: first estimates (until 2026-11-26 the Windows side's own,
 * windows/vp4u/vp4u_map.cpp), not measured against VisionPose or a Kinect v2. */
#define NECK_FRACTION 0.35f      /* NECK = shoulder centre + f * (head - shoulder centre) */
#define HAND_TIP_EXTENSION 1.2f  /* HAND_TIP = hand + e * (hand - wrist) */
#define THUMB_FRACTION 0.8f      /* THUMB = wrist + f * (hand - wrist) + THUMB_FORWARD along the body's forward */
#define THUMB_FORWARD 0.03f
#define NOSE_FORWARD 0.085f
#define NOSE_DOWN 0.02f
#define EYE_FORWARD 0.075f
#define EYE_UP 0.03f
#define EYE_SIDE 0.032f
#define EAR_SIDE 0.075f
#define EAR_BACK 0.01f

static uint8_t derived_state(uint8_t a, uint8_t b) { return (a && b) ? L2K_JOINT_INFERRED : L2K_JOINT_NOT_TRACKED; }

/* The extra joints placed from the skeleton's own joints; `up` is the frame's up vector. */
static void derive_extras(const l2k_skeleton *s, l2k_vec3 up, l2k_vec3 *p, uint8_t *st) {
    l2k_vec3 x_default = {1, 0, 0}, fwd_default = {0, 0, -1};
    /* the player's right, level, and where they face (towards the sensor for a player facing it: -z) */
    l2k_vec3 side = v_sub(s->joints[L2K_JOINT_SHOULDER_RIGHT], s->joints[L2K_JOINT_SHOULDER_LEFT]);
    l2k_vec3 fwd, head = s->joints[L2K_JOINT_HEAD], sc = s->joints[L2K_JOINT_SHOULDER_CENTER];
    uint8_t face = s->joint_state[L2K_JOINT_HEAD] ? L2K_JOINT_INFERRED : L2K_JOINT_NOT_TRACKED;
    int i;
    side = v_unit(v_sub(side, v_scale(up, v_dot(side, up))), x_default);
    fwd = v_unit(v_cross(up, side), fwd_default);

    p[0] = v_add(sc, v_scale(v_sub(head, sc), NECK_FRACTION));
    st[0] = derived_state(s->joint_state[L2K_JOINT_SHOULDER_CENTER], s->joint_state[L2K_JOINT_HEAD]);
    for (i = 0; i < 2; i++) {
        int wrist = i ? L2K_JOINT_WRIST_RIGHT : L2K_JOINT_WRIST_LEFT, hand = i ? L2K_JOINT_HAND_RIGHT : L2K_JOINT_HAND_LEFT;
        l2k_vec3 w = s->joints[wrist], h = s->joints[hand], wh = v_sub(h, w);
        uint8_t ds = derived_state(s->joint_state[wrist], s->joint_state[hand]);
        p[1 + 2 * i] = v_add(h, v_scale(wh, HAND_TIP_EXTENSION));
        p[2 + 2 * i] = v_add(v_add(w, v_scale(wh, THUMB_FRACTION)), v_scale(fwd, THUMB_FORWARD));
        st[1 + 2 * i] = st[2 + 2 * i] = ds;
    }
    p[5] = v_add(head, v_sub(v_scale(fwd, NOSE_FORWARD), v_scale(up, NOSE_DOWN)));
    p[6] = v_add(head, v_add(v_add(v_scale(fwd, EYE_FORWARD), v_scale(up, EYE_UP)), v_scale(side, -EYE_SIDE)));
    p[7] = v_add(head, v_add(v_add(v_scale(fwd, EYE_FORWARD), v_scale(up, EYE_UP)), v_scale(side, EYE_SIDE)));
    p[8] = v_add(head, v_add(v_scale(side, -EAR_SIDE), v_scale(fwd, -EAR_BACK)));
    p[9] = v_add(head, v_add(v_scale(side, EAR_SIDE), v_scale(fwd, -EAR_BACK)));
    for (i = 5; i < L2K_BODY_EXTRA_COUNT; i++) st[i] = face;
}

static uint8_t map_body(const l2k_map_params *prm, const l2k_src_body *b, int quadrant, l2k_vec3 up, l2k_skeleton *s, joint_memory *mem,
                        l2k_vec3 *extra_p, uint8_t *extra_state) {
    l2k_vec3 k[L2K_SRC_JOINT_COUNT];
    uint8_t conf[L2K_SRC_JOINT_COUNT];
    l2k_vec3 pelvis, neck, torso, sum = {0, 0, 0}, shoulder_line = {1, 0, 0};
    float shoulder_width = 0.38f; /* used only if upper-body mode has a pelvis but no shoulders */
    uint8_t pelvis_conf, core_conf;
    int i, known = 0, have_shoulders = 0;
    size_t d;

    for (i = 0; i < L2K_SRC_JOINT_COUNT; i++) {
        conf[i] = b->conf[i] > 2 ? 2 : b->conf[i];
        k[i] = l2k_camera_to_kinect(b->p[i], quadrant);
        if (conf[i] && i < L2K_SRC_SKELETON_COUNT) {
            sum = v_add(sum, k[i]);
            known++;
        }
    }
    if (!known) return L2K_SKELETON_NOT_TRACKED;

    if (prm->upper_body) {
        /* upper-body mode: the legs are never the source's (they are filled in below) */
        static const uint8_t LEGS[8] = {L2K_SRC_HIP_LEFT, L2K_SRC_KNEE_LEFT, L2K_SRC_ANKLE_LEFT, L2K_SRC_TOE_LEFT,
                                        L2K_SRC_HIP_RIGHT, L2K_SRC_KNEE_RIGHT, L2K_SRC_ANKLE_RIGHT, L2K_SRC_TOE_RIGHT};
        for (i = 0; i < 8; i++) conf[LEGS[i]] = 0;
        if (conf[L2K_SRC_SHOULDER_LEFT] && conf[L2K_SRC_SHOULDER_RIGHT]) {
            shoulder_line = v_sub(k[L2K_SRC_SHOULDER_RIGHT], k[L2K_SRC_SHOULDER_LEFT]);
            shoulder_width = v_len(shoulder_line);
            have_shoulders = 1;
        }
    }

    pelvis = k[L2K_SRC_PELVIS];
    pelvis_conf = conf[L2K_SRC_PELVIS];
    if (!pelvis_conf && conf[L2K_SRC_HIP_LEFT] && conf[L2K_SRC_HIP_RIGHT]) {
        pelvis = v_scale(v_add(k[L2K_SRC_HIP_LEFT], k[L2K_SRC_HIP_RIGHT]), 0.5f);
        pelvis_conf = u8_min(conf[L2K_SRC_HIP_LEFT], conf[L2K_SRC_HIP_RIGHT]);
    }
    if (!pelvis_conf && prm->upper_body && have_shoulders && conf[L2K_SRC_NECK]) {
        /* no pelvis in view: straight below the neck, a torso length away */
        pelvis = v_sub(k[L2K_SRC_NECK], v_scale(up, prm->upper_torso_ratio * shoulder_width));
        pelvis_conf = L2K_JOINT_INFERRED;
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

    /* upper-body mode: a standing lower body, straight down from the pelvis */
    if (prm->upper_body) {
        l2k_vec3 x_default = {1, 0, 0}, fwd_default = {0, 0, -1};
        /* the player's right, level: the shoulder line without its vertical part */
        l2k_vec3 right = v_unit(v_sub(shoulder_line, v_scale(up, v_dot(shoulder_line, up))), x_default);
        l2k_vec3 facing = v_unit(v_cross(up, right), fwd_default);
        float hip_half = prm->upper_hip_ratio * shoulder_width;
        float leg = prm->upper_leg_ratio * v_len(torso);
        for (i = 0; i < 2; i++) {
            int hip = i ? L2K_JOINT_HIP_RIGHT : L2K_JOINT_HIP_LEFT;
            l2k_vec3 h = v_add(pelvis, v_scale(right, i ? hip_half : -hip_half));
            l2k_vec3 a = v_sub(h, v_scale(up, leg));
            s->joints[hip] = h;
            s->joints[hip + 1] = v_scale(v_add(h, a), 0.5f); /* knee */
            s->joints[hip + 2] = a;                          /* ankle */
            s->joints[hip + 3] = v_add(a, v_sub(v_scale(facing, prm->foot_length), v_scale(up, prm->foot_drop))); /* foot */
            s->joint_state[hip] = s->joint_state[hip + 1] = s->joint_state[hip + 2] = s->joint_state[hip + 3] = L2K_JOINT_INFERRED;
        }
    }

    /* A joint the source did not give is inferred: it keeps its last known offset from its parent
     * joint (it sits on the parent if it was never seen). Parents come before children in joint order. */
    for (i = 1; i < L2K_JOINT_COUNT; i++) {
        if (s->joint_state[i] == L2K_JOINT_NOT_TRACKED) {
            s->joints[i] = s->joints[PARENT[i]];
            if (mem->valid[i]) s->joints[i] = v_add(s->joints[i], mem->offset[i]);
            s->joint_state[i] = L2K_JOINT_INFERRED;
        } else {
            mem->offset[i] = v_sub(s->joints[i], s->joints[PARENT[i]]);
            mem->valid[i] = 1;
        }
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

    /* the 30-joint body's other joints: the source's where it gives them, else placed from the skeleton */
    derive_extras(s, up, extra_p, extra_state);
    for (i = 0; i < L2K_BODY_EXTRA_COUNT; i++) {
        if (EXTRA_SRC[i] >= 0 && conf[EXTRA_SRC[i]]) {
            extra_p[i] = k[EXTRA_SRC[i]];
            extra_state[i] = conf[EXTRA_SRC[i]];
        }
    }
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

/* Roughly where a source body is, in skeleton space (to match it to a remembered player). */
static int rough_position(const l2k_src_body *b, int quadrant, l2k_vec3 *out) {
    static const uint8_t ORDER[4] = {L2K_SRC_PELVIS, L2K_SRC_NECK, L2K_SRC_HEAD, L2K_SRC_SHOULDER_LEFT};
    l2k_vec3 sum = {0, 0, 0};
    int i, known = 0;
    for (i = 0; i < 4; i++) {
        if (b->conf[ORDER[i]]) {
            *out = l2k_camera_to_kinect(b->p[ORDER[i]], quadrant);
            return 1;
        }
    }
    for (i = 0; i < L2K_SRC_SKELETON_COUNT; i++) {
        if (b->conf[i]) {
            sum = v_add(sum, l2k_camera_to_kinect(b->p[i], quadrant));
            known++;
        }
    }
    if (!known) return 0;
    *out = v_scale(sum, 1.0f / (float)known);
    return 1;
}

static uint32_t new_out_id(l2k_mapper *m) {
    if (++m->next_out_id == 0) m->next_out_id = 1;
    return m->next_out_id;
}

/* Move slot s's skeleton and its extra joints. */
static void shift_slot(l2k_mapper *m, l2k_skeleton_frame *out, int s, l2k_vec3 shift) {
    l2k_skeleton *sk = &out->skeletons[s];
    int j;
    sk->position = v_add(sk->position, shift);
    for (j = 0; j < L2K_JOINT_COUNT; j++) sk->joints[j] = v_add(sk->joints[j], shift);
    for (j = 0; j < L2K_BODY_EXTRA_COUNT; j++) m->extra[s].p[j] = v_add(m->extra[s].p[j], shift);
}

/* Turn p about the sensor by the rotation that takes the unit vector `up` to the y axis. */
static l2k_vec3 level_point(l2k_vec3 p, l2k_vec3 up) {
    l2k_vec3 axis = {-up.z, 0.0f, up.x}; /* up x (0,1,0) */
    float s = v_len(axis), c = up.y;
    l2k_vec3 k, r;
    if (s < 1e-6f) return p;
    k = v_scale(axis, 1.0f / s);
    r = v_add(v_add(v_scale(p, c), v_scale(v_cross(k, p), s)), v_scale(k, v_dot(k, p) * (1.0f - c)));
    return r;
}

/* See "Normalising" in l2k_map.h. */
static void normalize_frame(l2k_mapper *m, l2k_skeleton_frame *out, l2k_vec3 up, uint64_t timestamp_us) {
    const l2k_map_params *p = &m->params;
    int chosen[2] = {-1, -1}, count = 0, wanted = p->norm_players >= 2 ? 2 : 1, s, j, i;
    float dt = 1.0f / 30.0f, alpha;

    if (m->norm_last_us && timestamp_us > m->norm_last_us) {
        dt = (float)(timestamp_us - m->norm_last_us) * 1e-6f;
        if (dt > 0.5f) dt = 0.5f;
    }
    m->norm_last_us = timestamp_us;
    alpha = p->norm_follow_seconds > 0.0f ? 1.0f - expf(-dt / p->norm_follow_seconds) : 1.0f;

    /* a level sensor */
    for (s = 0; s < L2K_SKELETON_SLOTS; s++) {
        l2k_skeleton *sk = &out->skeletons[s];
        if (sk->state == L2K_SKELETON_NOT_TRACKED) continue;
        sk->position = level_point(sk->position, up);
        for (j = 0; j < L2K_JOINT_COUNT; j++) sk->joints[j] = level_point(sk->joints[j], up);
        for (j = 0; j < L2K_BODY_EXTRA_COUNT; j++) m->extra[s].p[j] = level_point(m->extra[s].p[j], up);
    }
    out->up.x = 0.0f; out->up.y = 1.0f; out->up.z = 0.0f;
    if (out->flags & L2K_FRAME_FLOOR_VALID) {
        out->floor_plane.x = 0.0f; out->floor_plane.y = 1.0f; out->floor_plane.z = 0.0f;
    }

    /* the players: those whose identity is the oldest */
    for (s = 0; s < L2K_SKELETON_SLOTS; s++) {
        if (out->skeletons[s].state == L2K_SKELETON_NOT_TRACKED) continue;
        for (i = 0; i < count; i++) {
            if (m->out_id[s] < m->out_id[chosen[i]]) break;
        }
        if (i >= wanted) continue;
        for (j = (count < wanted ? count : wanted - 1); j > i; j--) chosen[j] = chosen[j - 1];
        chosen[i] = s;
        if (count < wanted) count++;
    }
    for (s = 0; s < L2K_SKELETON_SLOTS; s++) {
        if (s == chosen[0] || s == chosen[1]) continue;
        if (out->skeletons[s].state != L2K_SKELETON_NOT_TRACKED) memset(&out->skeletons[s], 0, sizeof out->skeletons[s]);
        memset(&m->extra[s], 0, sizeof m->extra[s]);
        m->anchor[s].valid = 0;
        m->anchor[s].side = 0;
    }

    /* each player's standing point follows where the player is */
    for (i = 0; i < count; i++) {
        l2k_skeleton *sk = &out->skeletons[chosen[i]];
        float dx, dz, d;
        s = chosen[i];
        if (!m->anchor[s].valid) {
            m->anchor[s].x = sk->position.x;
            m->anchor[s].z = sk->position.z;
            m->anchor[s].valid = 1;
            m->anchor[s].side = 0;
        } else {
            m->anchor[s].x += (sk->position.x - m->anchor[s].x) * alpha;
            m->anchor[s].z += (sk->position.z - m->anchor[s].z) * alpha;
        }
        dx = sk->position.x - m->anchor[s].x;
        dz = sk->position.z - m->anchor[s].z;
        d = sqrtf(dx * dx + dz * dz);
        if (d > p->norm_reach && d > 0.0f) { /* a long stride: the standing point comes along */
            m->anchor[s].x = sk->position.x - dx * p->norm_reach / d;
            m->anchor[s].z = sk->position.z - dz * p->norm_reach / d;
        }
    }

    /* two players: each keeps the side they stand on */
    if (wanted == 2) {
        if (count == 2 && (m->anchor[chosen[0]].side == 0 || m->anchor[chosen[1]].side == 0 ||
                           m->anchor[chosen[0]].side == m->anchor[chosen[1]].side)) {
            int first_is_lower = m->anchor[chosen[0]].x <= m->anchor[chosen[1]].x;
            if (m->anchor[chosen[0]].side != 0 && m->anchor[chosen[1]].side == 0) {
                m->anchor[chosen[1]].side = (int8_t)-m->anchor[chosen[0]].side; /* the newcomer takes the free side */
            } else if (m->anchor[chosen[1]].side != 0 && m->anchor[chosen[0]].side == 0) {
                m->anchor[chosen[0]].side = (int8_t)-m->anchor[chosen[1]].side;
            } else {
                m->anchor[chosen[0]].side = (int8_t)(first_is_lower ? -1 : 1);
                m->anchor[chosen[1]].side = (int8_t)(first_is_lower ? 1 : -1);
            }
        } else if (count == 1 && m->anchor[chosen[0]].side == 0) {
            m->anchor[chosen[0]].side = (int8_t)(m->anchor[chosen[0]].x < 0.0f ? -1 : 1);
        }
    }

    for (i = 0; i < count; i++) {
        l2k_vec3 shift;
        s = chosen[i];
        shift.x = (wanted == 2 ? (float)m->anchor[s].side * p->norm_spacing : 0.0f) - m->anchor[s].x;
        shift.y = 0.0f;
        shift.z = p->norm_distance - m->anchor[s].z;
        shift_slot(m, out, s, shift);
    }
}

void l2k_mapper_map(l2k_mapper *m, const l2k_src_body *bodies, int body_count, l2k_vec3 up_camera,
                    int quadrant, uint64_t timestamp_us, uint8_t source_kind, l2k_skeleton_frame *out) {
    l2k_vec3 up_default = {0, 1, 0};
    l2k_vec3 up = v_unit(l2k_camera_to_kinect(up_camera, quadrant), up_default);
    uint32_t ids[L2K_SKELETON_SLOTS];
    int slot_of[L2K_SKELETON_SLOTS];
    int i, j, s, n = body_count > L2K_SKELETON_SLOTS ? L2K_SKELETON_SLOTS : body_count;
    int have_floor = 0;
    float floor_h = 0.0f;
    uint64_t hold_us = (uint64_t)((m->params.hold_seconds > 0.0f ? m->params.hold_seconds : 0.0f) * 1e6f);
    uint64_t identity_us = (uint64_t)((m->params.identity_seconds > 0.0f ? m->params.identity_seconds : 0.0f) * 1e6f);

    memset(out, 0, sizeof *out);
    memset(m->extra, 0, sizeof m->extra);
    out->timestamp_us = timestamp_us;
    out->frame_number = ++m->frame_number;
    out->source_kind = source_kind;
    out->up = up;
    if (m->params.upper_body) out->flags |= L2K_FRAME_UPPER_BODY;

    /* 1. a body keeps its slot while the source keeps its id */
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
        if (!keep) m->slot_id[s] = 0; /* gone from the source; the identity stays for now */
    }

    /* 2. identities that have been absent too long are forgotten */
    for (s = 0; s < L2K_SKELETON_SLOTS; s++) {
        uint64_t absent;
        int expired;
        if (m->slot_id[s] || !m->out_id[s]) continue;
        absent = timestamp_us > m->last_seen_us[s] ? timestamp_us - m->last_seen_us[s] : 0;
        if (m->params.keep_identity) expired = identity_us > 0 && absent > identity_us;
        else expired = absent > hold_us;
        if (expired) {
            m->out_id[s] = 0;
            m->has_last[s] = 0;
            memset(&m->memory[s], 0, sizeof m->memory[s]);
        }
    }

    /* 3. a body the source newly reports: the remembered player standing nearest, else a fresh slot */
    for (i = 0; i < n; i++) {
        l2k_vec3 pos;
        int best = -1, have_pos;
        float best_d = 0.0f;
        if (slot_of[i] >= 0) continue;
        have_pos = rough_position(&bodies[i], quadrant, &pos);
        for (s = 0; s < L2K_SKELETON_SLOTS; s++) {
            float d;
            if (m->slot_id[s] || !m->out_id[s]) continue;
            d = (have_pos && m->has_last[s]) ? v_len(v_sub(pos, m->last[s].position)) : 1e9f;
            if (best < 0 || d < best_d) {
                best = s;
                best_d = d;
            }
        }
        if (best < 0) {
            for (s = 0; s < L2K_SKELETON_SLOTS; s++) {
                if (!m->slot_id[s] && !m->out_id[s]) {
                    best = s;
                    m->out_id[s] = new_out_id(m);
                    m->has_last[s] = 0;
                    memset(&m->memory[s], 0, sizeof m->memory[s]); /* a new player: no remembered pose */
                    break;
                }
            }
        }
        if (best < 0) continue; /* six players already */
        m->slot_id[best] = ids[i];
        slot_of[i] = best;
    }

    /* 4. the skeletons */
    for (i = 0; i < n; i++) {
        l2k_skeleton *sk;
        float h;
        if (slot_of[i] < 0) continue;
        s = slot_of[i];
        sk = &out->skeletons[s];
        sk->state = map_body(&m->params, &bodies[i], quadrant, up, sk, (joint_memory *)(void *)&m->memory[s],
                             m->extra[s].p, m->extra[s].state);
        if (sk->state == L2K_SKELETON_NOT_TRACKED) {
            m->slot_id[s] = 0;
            continue;
        }
        sk->tracking_id = m->out_id[s];
        if (sk->state != L2K_SKELETON_TRACKED) continue;
        m->last[s] = *sk;
        m->last_extra[s] = m->extra[s];
        m->has_last[s] = 1;
        m->last_seen_us[s] = timestamp_us;
        if (floor_under(&m->params, sk, up, &h)) {
            if (!have_floor || h < floor_h) floor_h = h;
            have_floor = 1;
        }
    }

    /* 5. a player not tracked in this frame is held at the last skeleton for a short while */
    for (s = 0; s < L2K_SKELETON_SLOTS; s++) {
        uint64_t absent;
        if (out->skeletons[s].state == L2K_SKELETON_TRACKED || !m->out_id[s] || !m->has_last[s]) continue;
        absent = timestamp_us > m->last_seen_us[s] ? timestamp_us - m->last_seen_us[s] : 0;
        if (absent > hold_us) continue;
        out->skeletons[s] = m->last[s];
        m->extra[s] = m->last_extra[s];
        for (j = 0; j < L2K_JOINT_COUNT; j++) {
            if (out->skeletons[s].joint_state[j] > L2K_JOINT_INFERRED) out->skeletons[s].joint_state[j] = L2K_JOINT_INFERRED;
        }
        for (j = 0; j < L2K_BODY_EXTRA_COUNT; j++) {
            if (m->extra[s].state[j] > L2K_JOINT_INFERRED) m->extra[s].state[j] = L2K_JOINT_INFERRED;
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
    /* play area, vertically: the floor the game is told about must put the first player's hip at a human height */
    if ((m->params.fit_play_area || m->params.normalize) && !m->floor_fixed) {
        for (s = 0; s < L2K_SKELETON_SLOTS; s++) {
            float hip;
            if (out->skeletons[s].state != L2K_SKELETON_TRACKED) continue;
            hip = v_dot(up, out->skeletons[s].position); /* relative to the sensor, along up */
            if (!m->floor_valid) {
                m->sensor_height = 0.5f * (m->params.area_hip_min + m->params.area_hip_max) - hip;
                m->floor_valid = 1;
            } else if (hip + m->sensor_height < m->params.area_hip_min) {
                m->sensor_height = m->params.area_hip_min - hip;
            } else if (hip + m->sensor_height > m->params.area_hip_max) {
                m->sensor_height = m->params.area_hip_max - hip;
            }
            break;
        }
    }
    if (m->floor_valid) {
        out->flags |= L2K_FRAME_FLOOR_VALID;
        out->floor_plane.x = up.x;
        out->floor_plane.y = up.y;
        out->floor_plane.z = up.z;
        out->floor_plane.w = m->sensor_height;
    }

    /* 6. where the game is told the players are */
    for (s = 0; s < L2K_SKELETON_SLOTS; s++) m->picture[s] = out->skeletons[s];
    if (m->params.normalize) {
        normalize_frame(m, out, up, timestamp_us);
    } else if (m->params.fit_play_area) {
        /* play area: a body outside it slides horizontally to just inside */
        l2k_vec3 z_axis = {0, 0, 1}, x_default = {1, 0, 0};
        l2k_vec3 forward = v_unit(v_sub(z_axis, v_scale(up, v_dot(z_axis, up))), z_axis); /* level, away from the sensor */
        l2k_vec3 side = v_unit(v_cross(up, forward), x_default);
        for (s = 0; s < L2K_SKELETON_SLOTS; s++) {
            l2k_skeleton *sk = &out->skeletons[s];
            float d, x, d2, x2;
            if (sk->state == L2K_SKELETON_NOT_TRACKED) continue;
            d = v_dot(sk->position, forward);
            x = v_dot(sk->position, side);
            d2 = d < m->params.area_near ? m->params.area_near : d > m->params.area_far ? m->params.area_far : d;
            x2 = x < -m->params.area_half_width ? -m->params.area_half_width : x > m->params.area_half_width ? m->params.area_half_width : x;
            if (d2 == d && x2 == x) continue;
            shift_slot(m, out, s, v_add(v_scale(forward, d2 - d), v_scale(side, x2 - x)));
        }
    }
    if (!m->params.normalize) {
        for (s = 0; s < L2K_SKELETON_SLOTS; s++) m->anchor[s].valid = 0;
        m->norm_last_us = 0;
    }
}

/* ---- the 30-joint body ---- */

/* One slot; `extra` (may be NULL) holds the mapper's extra joints for it, used where their state is nonzero. */
static void body_of(const l2k_skeleton *sk, l2k_vec3 up, const l2k_vec3 *extra_p, const uint8_t *extra_state, l2k_body *out) {
    l2k_vec3 p[L2K_BODY_EXTRA_COUNT];
    uint8_t st[L2K_BODY_EXTRA_COUNT];
    int j;
    memset(out, 0, sizeof *out);
    out->state = sk->state;
    out->quality = sk->quality;
    out->tracking_id = sk->tracking_id;
    out->position = sk->position;
    if (sk->state != L2K_SKELETON_TRACKED) return;
    for (j = 0; j < L2K_JOINT_COUNT; j++) {
        int to = j == L2K_JOINT_SHOULDER_CENTER ? L2K_BODY_SPINE_SHOULDER : j;
        out->joints[to] = sk->joints[j];
        out->joint_state[to] = sk->joint_state[j];
    }
    derive_extras(sk, up, p, st);
    for (j = 0; j < L2K_BODY_EXTRA_COUNT; j++) {
        if (extra_p && extra_state && extra_state[j]) {
            p[j] = extra_p[j];
            st[j] = extra_state[j];
        }
        out->joints[EXTRA_BODY[j]] = p[j];
        out->joint_state[EXTRA_BODY[j]] = st[j];
    }
}

static void body_frame_head(const l2k_skeleton_frame *f, l2k_body_frame *out) {
    memset(out, 0, sizeof *out);
    out->timestamp_us = f->timestamp_us;
    out->frame_number = f->frame_number;
    out->flags = f->flags;
    out->floor_plane = f->floor_plane;
    out->up = f->up;
    out->source_kind = f->source_kind;
}

void l2k_body_from_skeleton(const l2k_skeleton *skeleton, l2k_vec3 up, l2k_body *out) {
    body_of(skeleton, up, NULL, NULL, out);
}

void l2k_body_frame_from_skeleton(const l2k_skeleton_frame *frame, l2k_body_frame *out) {
    int s;
    body_frame_head(frame, out);
    for (s = 0; s < L2K_SKELETON_SLOTS; s++) body_of(&frame->skeletons[s], frame->up, NULL, NULL, &out->bodies[s]);
}

void l2k_mapper_body_frame(const l2k_mapper *m, const l2k_skeleton_frame *frame, l2k_body_frame *out) {
    int s, own = frame->frame_number == m->frame_number;
    body_frame_head(frame, out);
    for (s = 0; s < L2K_SKELETON_SLOTS; s++) {
        body_of(&frame->skeletons[s], frame->up, own ? m->extra[s].p : NULL, own ? m->extra[s].state : NULL, &out->bodies[s]);
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
