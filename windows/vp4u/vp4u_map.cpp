/* Kinect v1 skeleton -> vp4u bodies. Conventions: vp4u_map.h and windows/docs/vp4u-interface.md. */
#include "vp4u_map.h"

#include <math.h>
#include <string.h>

namespace vp4u {

namespace {

l2k_vec3 v3(float x, float y, float z) {
    l2k_vec3 r = {x, y, z};
    return r;
}
l2k_vec3 add(l2k_vec3 a, l2k_vec3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
l2k_vec3 sub(l2k_vec3 a, l2k_vec3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
l2k_vec3 mul(l2k_vec3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
l2k_vec3 lerp(l2k_vec3 a, l2k_vec3 b, float t) { return add(a, mul(sub(b, a), t)); }
float dot(l2k_vec3 a, l2k_vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
l2k_vec3 cross(l2k_vec3 a, l2k_vec3 b) { return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
float len(l2k_vec3 a) { return sqrtf(dot(a, a)); }
l2k_vec3 norm(l2k_vec3 a, l2k_vec3 fallback) {
    float l = len(a);
    return l > 1e-6f ? mul(a, 1.0f / l) : fallback;
}

/* Where the 20 Kinect v1 joints go (same index for 0..19: Kinect v2 kept v1's order there). */
const int kFromKinect[L2K_JOINT_COUNT] = {
    VP4U_SPINE_BASE,     /* HIP_CENTER */
    VP4U_SPINE_MID,      /* SPINE */
    VP4U_SPINE_SHOULDER, /* SHOULDER_CENTER: Kinect v1 puts it between the shoulders, like v2's SPINE_SHOULDER */
    VP4U_HEAD,
    VP4U_SHOULDER_LEFT, VP4U_ELBOW_LEFT, VP4U_WRIST_LEFT, VP4U_HAND_LEFT,
    VP4U_SHOULDER_RIGHT, VP4U_ELBOW_RIGHT, VP4U_WRIST_RIGHT, VP4U_HAND_RIGHT,
    VP4U_HIP_LEFT, VP4U_KNEE_LEFT, VP4U_ANKLE_LEFT, VP4U_FOOT_LEFT,
    VP4U_HIP_RIGHT, VP4U_KNEE_RIGHT, VP4U_ANKLE_RIGHT, VP4U_FOOT_RIGHT,
};

/* Placement of the joints Kinect v1 does not have. First estimates (docs/vp4u-interface.md
 * "Derived joints"), not measured against a VisionPose recording. */
const float kNeckFraction = 0.35f;     /* NECK = spine shoulder + f * (head - spine shoulder) */
const float kHandTipExtension = 1.2f;  /* HAND_TIP = hand + e * (hand - wrist) */
const float kThumbFraction = 0.8f;     /* THUMB = wrist + f * (hand - wrist) + forward * kThumbForward */
const float kThumbForward = 0.03f;
const float kNoseForward = 0.085f, kNoseDown = 0.02f;
const float kEyeForward = 0.075f, kEyeUp = 0.03f, kEyeSide = 0.032f;
const float kEarSide = 0.075f, kEarBack = 0.01f;

int derived_state(int a, int b) { return (a > 0 && b > 0) ? VP4U_INFERRED : VP4U_NOT_TRACKED; }

} // namespace

void map_params_default(MapParams *p) {
    p->phone_x = 0.55f;
    p->phone_z = 0.0f;
    p->phone_height = 0.0f; /* from the floor plane */
    p->default_phone_height = 1.0f;
    p->camera_height = 1.16f;
    p->yaw_deg = 0.0f;
    p->mirror_x = false;
}

l2k_vec3 level_point(l2k_vec3 up, l2k_vec3 v) {
    const l2k_vec3 y = v3(0, 1, 0);
    up = norm(up, y);
    l2k_vec3 axis = cross(up, y); /* sin(angle) * k */
    float s = len(axis), c = dot(up, y);
    if (s < 1e-6f) {
        if (c > 0) return v;
        return v3(v.x, -v.y, -v.z); /* upside down: half a turn about x */
    }
    l2k_vec3 k = mul(axis, 1.0f / s);
    /* Rodrigues */
    return add(add(mul(v, c), mul(cross(k, v), s)), mul(k, dot(k, v) * (1.0f - c)));
}

int map_frame(const l2k_skeleton_frame &in, const MapParams &p, vp4u_body *out, int max_out, float *height_used) {
    float height = p.phone_height;
    if (height <= 0) height = (in.flags & L2K_FRAME_FLOOR_VALID) && in.floor_plane.w > 0.1f ? in.floor_plane.w : p.default_phone_height;
    if (height_used) *height_used = height;

    const float yaw = p.yaw_deg * 3.14159265f / 180.0f, cy = cosf(yaw), sy = sinf(yaw);
    const l2k_vec3 offset = v3(p.phone_x, height - p.camera_height, p.phone_z);
    const float sx = p.mirror_x ? -1.0f : 1.0f;
    l2k_vec3 up = in.up;
    if (len(up) < 0.5f) up = v3(0, 1, 0);

    int n = 0;
    for (int slot = 0; slot < L2K_SKELETON_SLOTS && n < max_out; slot++) {
        const l2k_skeleton &k = in.skeletons[slot];
        if (k.state != L2K_SKELETON_TRACKED) continue;

        /* level + yaw, still with the phone at the origin. Kinect skeleton space and the game's
         * frame agree on the axes (+x the player's right when facing the sensor, +y up, +z towards
         * the player), so nothing else turns. */
        l2k_vec3 p20[L2K_JOINT_COUNT];
        int s20[L2K_JOINT_COUNT];
        for (int j = 0; j < L2K_JOINT_COUNT; j++) {
            l2k_vec3 q = level_point(up, k.joints[j]);
            p20[j] = v3(cy * q.x + sy * q.z, q.y, -sy * q.x + cy * q.z);
            s20[j] = k.joint_state[j];
        }

        l2k_vec3 pos[VP4U_JOINT_COUNT];
        int st[VP4U_JOINT_COUNT];
        memset(st, 0, sizeof st);
        for (int j = 0; j < L2K_JOINT_COUNT; j++) {
            pos[kFromKinect[j]] = p20[j];
            st[kFromKinect[j]] = s20[j];
        }

        /* body axes for the derived joints: side = the player's right, forward = where they face */
        const l2k_vec3 vup = v3(0, 1, 0);
        l2k_vec3 side = sub(p20[L2K_JOINT_SHOULDER_RIGHT], p20[L2K_JOINT_SHOULDER_LEFT]);
        side.y = 0;
        side = norm(side, v3(1, 0, 0));
        l2k_vec3 fwd = norm(cross(vup, side), v3(0, 0, -1));

        const l2k_vec3 sc = p20[L2K_JOINT_SHOULDER_CENTER], head = p20[L2K_JOINT_HEAD];
        const int s_sc = s20[L2K_JOINT_SHOULDER_CENTER], s_head = s20[L2K_JOINT_HEAD];
        pos[VP4U_NECK] = lerp(sc, head, kNeckFraction);
        st[VP4U_NECK] = derived_state(s_sc, s_head);

        struct HandSide {
            int wrist, hand, tip, thumb;
        } hands[2] = {{L2K_JOINT_WRIST_LEFT, L2K_JOINT_HAND_LEFT, VP4U_HAND_TIP_LEFT, VP4U_THUMB_LEFT},
                      {L2K_JOINT_WRIST_RIGHT, L2K_JOINT_HAND_RIGHT, VP4U_HAND_TIP_RIGHT, VP4U_THUMB_RIGHT}};
        for (const HandSide &h : hands) {
            l2k_vec3 w = p20[h.wrist], hd = p20[h.hand];
            int ds = derived_state(s20[h.wrist], s20[h.hand]);
            pos[h.tip] = add(hd, mul(sub(hd, w), kHandTipExtension));
            pos[h.thumb] = add(lerp(w, hd, kThumbFraction), mul(fwd, kThumbForward));
            st[h.tip] = st[h.thumb] = ds;
        }

        int fs = s_head > 0 ? VP4U_INFERRED : VP4U_NOT_TRACKED;
        pos[VP4U_NOSE] = add(head, add(mul(fwd, kNoseForward), mul(vup, -kNoseDown)));
        pos[VP4U_EYE_LEFT] = add(head, add(add(mul(fwd, kEyeForward), mul(vup, kEyeUp)), mul(side, -kEyeSide)));
        pos[VP4U_EYE_RIGHT] = add(head, add(add(mul(fwd, kEyeForward), mul(vup, kEyeUp)), mul(side, kEyeSide)));
        pos[VP4U_EAR_LEFT] = add(head, add(mul(side, -kEarSide), mul(fwd, -kEarBack)));
        pos[VP4U_EAR_RIGHT] = add(head, add(mul(side, kEarSide), mul(fwd, -kEarBack)));
        st[VP4U_NOSE] = st[VP4U_EYE_LEFT] = st[VP4U_EYE_RIGHT] = st[VP4U_EAR_LEFT] = st[VP4U_EAR_RIGHT] = fs;

        vp4u_body &b = out[n++];
        memset(&b, 0, sizeof b);
        b.state = VP4U_TRACKED;
        b.tracking_id = (int64_t)k.tracking_id;
        for (int j = 0; j < VP4U_JOINT_COUNT; j++) {
            l2k_vec3 g = add(v3(sx * pos[j].x, pos[j].y, pos[j].z), offset);
            b.joints[j].state = st[j];
            b.joints[j].x = g.x;
            b.joints[j].y = g.y;
            b.joints[j].z = g.z;
        }
    }
    return n;
}

} // namespace vp4u
