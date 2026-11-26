/* 30-joint body frame -> vp4u bodies. Conventions: vp4u_map.h and windows/docs/vp4u-interface.md. */
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
l2k_vec3 mul(l2k_vec3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
float dot(l2k_vec3 a, l2k_vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
l2k_vec3 cross(l2k_vec3 a, l2k_vec3 b) { return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
float len(l2k_vec3 a) { return sqrtf(dot(a, a)); }
l2k_vec3 norm(l2k_vec3 a, l2k_vec3 fallback) {
    float l = len(a);
    return l > 1e-6f ? mul(a, 1.0f / l) : fallback;
}

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

int map_body_frame(const l2k_body_frame &in, const MapParams &p, vp4u_body *out, int max_out, float *height_used) {
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
        const l2k_body &k = in.bodies[slot];
        if (k.state != L2K_SKELETON_TRACKED) continue;

        /* Kinect skeleton space and the game's frame agree on the axes (+x the player's right when facing
         * the sensor, +y up, +z towards the player), so only levelling, yaw and the offset remain. */
        vp4u_body &b = out[n++];
        memset(&b, 0, sizeof b);
        b.state = VP4U_TRACKED;
        b.tracking_id = (int64_t)k.tracking_id;
        for (int j = 0; j < VP4U_JOINT_COUNT; j++) {
            l2k_vec3 q = level_point(up, k.joints[j]);
            l2k_vec3 r = v3(cy * q.x + sy * q.z, q.y, -sy * q.x + cy * q.z);
            l2k_vec3 g = add(v3(sx * r.x, r.y, r.z), offset);
            b.joints[j].state = k.joint_state[j];
            b.joints[j].x = g.x;
            b.joints[j].y = g.y;
            b.joints[j].z = g.z;
        }
    }
    return n;
}

int map_frame(const l2k_skeleton_frame &in, const MapParams &p, vp4u_body *out, int max_out, float *height_used) {
    static thread_local l2k_body_frame body; /* ~2.5 kB */
    l2k_body_frame_from_skeleton(&in, &body);
    return map_body_frame(body, p, out, max_out, height_used);
}

} // namespace vp4u
