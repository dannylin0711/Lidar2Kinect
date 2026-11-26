/*
 * Kinect v1 skeleton frame (what the L2K1 protocol carries) -> DanceAround vp4u bodies.
 *
 * 1. Level: rotate the skeleton so the frame's `up` is +y. The phone's pitch and roll go; its yaw
 *    stays (plus `yaw_deg`).
 * 2. Place: put the phone where the game's coordinate origin expects a camera
 *    (windows/docs/vp4u-interface.md "Coordinates"): x += phone_x, y += phone height - camera
 *    height, z += phone_z.
 * 3. Joints: the 20 Kinect v1 joints map onto the first 20 vp4u joints; the other 10 (neck, spine
 *    shoulder, hand tips, thumbs, face) are placed from them and marked inferred.
 *
 * Pure functions, no Windows dependency, so tests/test_vp4u.cpp checks them directly.
 */
#ifndef VP4U_MAP_H
#define VP4U_MAP_H

#include "vp4u_abi.h"

extern "C" {
#include "l2k/l2k.h"
}

namespace vp4u {

struct MapParams {
    float phone_x;              /* m: the phone's position along the game's x axis (default 0.55, the play area's centre) */
    float phone_z;              /* m: the phone's position along the game's z axis (0 = where the cabinet's cameras were) */
    float phone_height;         /* m above the floor; <= 0 = from the frame's floor plane */
    float default_phone_height; /* used when phone_height <= 0 and the frame has no floor plane */
    float camera_height;        /* m above the floor of the game's coordinate origin (the cabinet's main camera) */
    float yaw_deg;              /* extra rotation about the vertical axis through the phone: +90 moves a point straight
                                 * ahead of the phone to the phone's +x side (the player's right) */
    bool mirror_x;              /* swap the player's left and right (for if docs/vp4u-interface.md's open point 1 is wrong) */
};

void map_params_default(MapParams *p);

/* Builds one vp4u body per tracked Kinect skeleton (position-only and empty slots are left out).
 * Returns the number written, at most max_out. `height_used` (optional) receives the phone height
 * the frame was placed with. */
int map_frame(const l2k_skeleton_frame &in, const MapParams &p, vp4u_body *out, int max_out, float *height_used = nullptr);

/* Rotation that takes `up` to +y (the shortest one). Applied to a point. Exposed for the tests. */
l2k_vec3 level_point(l2k_vec3 up, l2k_vec3 v);

} // namespace vp4u

#endif /* VP4U_MAP_H */
