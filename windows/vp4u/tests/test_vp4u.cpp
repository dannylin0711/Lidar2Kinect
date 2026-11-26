/*
 * Unit test of the Kinect v1 -> vp4u mapping (vp4u_map.cpp) with the shared synthetic figure:
 * axes, levelling of a tilted phone, the floor, the game's play area, the derived joints.
 */
#include "vp4u_map.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_checks, g_failed;
#define CHECK(c)                                                              \
    do {                                                                      \
        g_checks++;                                                           \
        if (!(c)) {                                                           \
            g_failed++;                                                       \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);               \
        }                                                                     \
    } while (0)
#define NEAR(a, b, tol) (fabsf((float)(a) - (float)(b)) <= (tol))

/* The game's play area (prop/udnappsetting.json motionCaptureSettings): centre, size, margin.
 * MotionCaptureSystem checks the SpineBase of the foremost body against it. */
static const float kAreaCenter[3] = {0.55f, 0.0f, 2.30f};
static const float kAreaHalf[3] = {0.8f + 0.1f, 1.0f + 0.1f, 0.405f + 0.1f};

static bool in_play_area(const vp4u_joint &j) {
    return fabsf(j.x - kAreaCenter[0]) <= kAreaHalf[0] && fabsf(j.y - kAreaCenter[1]) <= kAreaHalf[1] &&
           fabsf(j.z - kAreaCenter[2]) <= kAreaHalf[2];
}

/* Kinect frames of the synthetic figure seen by a phone `height` m up, tilted `tilt` degrees;
 * the mapper runs for a second first so its floor estimate settles. */
static l2k_skeleton_frame kinect_frame(int players, float height, float tilt, double t) {
    static l2k_mapper m;
    l2k_mapper_init(&m, NULL);
    l2k_skeleton_frame f;
    memset(&f, 0, sizeof f);
    for (int i = 0; i <= 30; i++) {
        double ti = t - (30 - i) / 30.0;
        l2k_src_body b[2];
        for (int p = 0; p < players; p++) l2k_synth_body_at(ti, p, players, height, tilt, &b[p]);
        l2k_mapper_map(&m, b, players, l2k_synth_up(tilt), 0, (uint64_t)(ti * 1e6), L2K_SOURCE_SYNTHETIC, &f);
    }
    return f;
}

static void check_body(const vp4u_body &b, const vp4u::MapParams &p, float floor_tol) {
    const vp4u_joint *j = b.joints;
    CHECK(b.state == VP4U_TRACKED);
    CHECK(b.tracking_id != 0);
    for (int i = 0; i < VP4U_JOINT_COUNT; i++) CHECK(j[i].state > 0);

    /* axes: +x is the player's right, +y up, +z away from the camera towards the player */
    CHECK(j[VP4U_SHOULDER_RIGHT].x > j[VP4U_SHOULDER_LEFT].x + 0.2f);
    CHECK(j[VP4U_HIP_RIGHT].x > j[VP4U_HIP_LEFT].x);
    CHECK(j[VP4U_EYE_RIGHT].x > j[VP4U_EYE_LEFT].x);
    CHECK(j[VP4U_HEAD].y > j[VP4U_NECK].y && j[VP4U_NECK].y > j[VP4U_SPINE_SHOULDER].y);
    CHECK(j[VP4U_SPINE_SHOULDER].y > j[VP4U_SPINE_MID].y && j[VP4U_SPINE_MID].y > j[VP4U_SPINE_BASE].y);
    CHECK(j[VP4U_KNEE_LEFT].y > j[VP4U_ANKLE_LEFT].y);
    CHECK(j[VP4U_NOSE].z < j[VP4U_HEAD].z); /* the figure faces the camera */
    CHECK(j[VP4U_EAR_LEFT].x < j[VP4U_HEAD].x && j[VP4U_EAR_RIGHT].x > j[VP4U_HEAD].x);

    /* the floor is camera_height below the origin: ankles about 7 cm above it */
    float floor_y = -p.camera_height;
    CHECK(NEAR(j[VP4U_ANKLE_LEFT].y - floor_y, 0.07f, floor_tol));
    CHECK(NEAR(j[VP4U_ANKLE_RIGHT].y - floor_y, 0.07f, floor_tol));
    /* a 1.7 m figure: head about 1.6 m above the floor */
    CHECK(NEAR(j[VP4U_HEAD].y - floor_y, 1.6f, 0.15f));
    /* depth is horizontal: head and ankles at about the same z although the phone may be tilted */
    CHECK(NEAR(j[VP4U_HEAD].z, j[VP4U_ANKLE_LEFT].z, 0.25f));

    /* derived joints stay near their parents */
    float tip = sqrtf(powf(j[VP4U_HAND_TIP_LEFT].x - j[VP4U_HAND_LEFT].x, 2) + powf(j[VP4U_HAND_TIP_LEFT].y - j[VP4U_HAND_LEFT].y, 2) +
                      powf(j[VP4U_HAND_TIP_LEFT].z - j[VP4U_HAND_LEFT].z, 2));
    CHECK(tip > 0.02f && tip < 0.2f);
}

int main() {
    vp4u::MapParams p;
    vp4u::map_params_default(&p);
    vp4u_body out[VP4U_MAX_BODIES];
    float h = 0;

    /* level, the shared synthetic scene: phone 1 m up */
    {
        l2k_skeleton_frame f = kinect_frame(1, 1.0f, 0.0f, 3.0);
        int n = vp4u::map_frame(f, p, out, VP4U_MAX_BODIES, &h);
        CHECK(n == 1);
        CHECK(NEAR(h, 1.0f, 0.06f)); /* from the floor plane */
        check_body(out[0], p, 0.06f);
        CHECK(in_play_area(out[0].joints[VP4U_SPINE_BASE]));
        CHECK(NEAR(out[0].joints[VP4U_SPINE_BASE].x, p.phone_x, 0.25f));
        CHECK(NEAR(out[0].joints[VP4U_SPINE_BASE].z, 2.5f, 0.3f));
    }

    /* tilted like the Dance Evolution cabinet (2 m, -24 deg): levelled to the same place */
    {
        l2k_skeleton_frame lv = kinect_frame(1, 1.0f, 0.0f, 3.0);
        l2k_skeleton_frame tl = kinect_frame(1, 2.0f, -24.0f, 3.0);
        vp4u_body a[VP4U_MAX_BODIES], b[VP4U_MAX_BODIES];
        CHECK(vp4u::map_frame(lv, p, a, VP4U_MAX_BODIES) == 1);
        CHECK(vp4u::map_frame(tl, p, b, VP4U_MAX_BODIES, &h) == 1);
        CHECK(NEAR(h, 2.0f, 0.08f));
        check_body(b[0], p, 0.08f);
        for (int j = 0; j < VP4U_JOINT_COUNT; j++) {
            CHECK(NEAR(a[0].joints[j].x, b[0].joints[j].x, 0.05f));
            CHECK(NEAR(a[0].joints[j].y, b[0].joints[j].y, 0.08f));
            CHECK(NEAR(a[0].joints[j].z, b[0].joints[j].z, 0.05f));
        }
        CHECK(in_play_area(b[0].joints[VP4U_SPINE_BASE]));
    }

    /* a fixed phone height overrides the floor plane */
    {
        vp4u::MapParams q = p;
        q.phone_height = 1.5f;
        l2k_skeleton_frame f = kinect_frame(1, 1.0f, 0.0f, 3.0);
        vp4u_body a[VP4U_MAX_BODIES], b[VP4U_MAX_BODIES];
        vp4u::map_frame(f, p, a, VP4U_MAX_BODIES);
        vp4u::map_frame(f, q, b, VP4U_MAX_BODIES, &h);
        CHECK(NEAR(h, 1.5f, 1e-4f));
        CHECK(NEAR(b[0].joints[VP4U_HEAD].y - a[0].joints[VP4U_HEAD].y, 0.5f, 0.06f));
    }

    /* no floor plane and no phone height: default_phone_height */
    {
        l2k_skeleton_frame f = kinect_frame(1, 1.0f, 0.0f, 3.0);
        f.flags &= ~L2K_FRAME_FLOOR_VALID;
        vp4u::map_frame(f, p, out, VP4U_MAX_BODIES, &h);
        CHECK(NEAR(h, p.default_phone_height, 1e-4f));
    }

    /* mirror_x swaps left and right; phone_x / phone_z move the body */
    {
        vp4u::MapParams q = p;
        q.mirror_x = true;
        q.phone_x = 0.0f;
        q.phone_z = 0.4f;
        l2k_skeleton_frame f = kinect_frame(1, 1.0f, 0.0f, 3.0);
        vp4u_body a[VP4U_MAX_BODIES], b[VP4U_MAX_BODIES];
        vp4u::map_frame(f, p, a, VP4U_MAX_BODIES);
        vp4u::map_frame(f, q, b, VP4U_MAX_BODIES);
        CHECK(b[0].joints[VP4U_SHOULDER_RIGHT].x < b[0].joints[VP4U_SHOULDER_LEFT].x);
        CHECK(NEAR(b[0].joints[VP4U_HEAD].x, -(a[0].joints[VP4U_HEAD].x - p.phone_x), 1e-4f));
        CHECK(NEAR(b[0].joints[VP4U_HEAD].z, a[0].joints[VP4U_HEAD].z + 0.4f, 1e-4f));
    }

    /* yaw 90 turns the body about the vertical axis through the phone */
    {
        vp4u::MapParams q = p;
        q.yaw_deg = 90.0f;
        l2k_skeleton_frame f = kinect_frame(1, 1.0f, 0.0f, 3.0);
        vp4u_body a[VP4U_MAX_BODIES], b[VP4U_MAX_BODIES];
        vp4u::map_frame(f, p, a, VP4U_MAX_BODIES);
        vp4u::map_frame(f, q, b, VP4U_MAX_BODIES);
        float ax = a[0].joints[VP4U_SPINE_BASE].x - p.phone_x, az = a[0].joints[VP4U_SPINE_BASE].z - p.phone_z;
        float bx = b[0].joints[VP4U_SPINE_BASE].x - p.phone_x, bz = b[0].joints[VP4U_SPINE_BASE].z - p.phone_z;
        CHECK(NEAR(sqrtf(ax * ax + az * az), sqrtf(bx * bx + bz * bz), 1e-3f));
        CHECK(NEAR(bx, az, 1e-3f) && NEAR(bz, -ax, 1e-3f));
        CHECK(NEAR(a[0].joints[VP4U_HEAD].y, b[0].joints[VP4U_HEAD].y, 1e-4f));
    }

    /* two players: two bodies, distinct ids; position-only / empty slots are left out */
    {
        l2k_skeleton_frame f = kinect_frame(2, 1.0f, 0.0f, 3.0);
        int n = vp4u::map_frame(f, p, out, VP4U_MAX_BODIES);
        CHECK(n == 2);
        if (n == 2) CHECK(out[0].tracking_id != out[1].tracking_id);
        for (int s = 0; s < L2K_SKELETON_SLOTS; s++)
            if (f.skeletons[s].state == L2K_SKELETON_TRACKED) {
                f.skeletons[s].state = L2K_SKELETON_POSITION_ONLY;
                break;
            }
        CHECK(vp4u::map_frame(f, p, out, VP4U_MAX_BODIES) == 1);
        CHECK(vp4u::map_frame(f, p, out, 0) == 0);
        memset(&f.skeletons, 0, sizeof f.skeletons);
        CHECK(vp4u::map_frame(f, p, out, VP4U_MAX_BODIES) == 0);
    }

    /* levelling itself */
    {
        l2k_vec3 up = {0.0f, cosf(0.4f), -sinf(0.4f)}, v = up;
        l2k_vec3 r = vp4u::level_point(up, v);
        CHECK(NEAR(r.x, 0, 1e-5f) && NEAR(r.y, 1, 1e-5f) && NEAR(r.z, 0, 1e-5f));
        l2k_vec3 y = {0, 1, 0}, w = {0.3f, -0.2f, 2.0f};
        r = vp4u::level_point(y, w);
        CHECK(NEAR(r.x, w.x, 1e-6f) && NEAR(r.y, w.y, 1e-6f) && NEAR(r.z, w.z, 1e-6f));
        l2k_vec3 down = {0, -1, 0};
        r = vp4u::level_point(down, w);
        CHECK(NEAR(r.y, 0.2f, 1e-6f));
    }

    printf("%d checks, %d failed\n", g_checks, g_failed);
    return g_failed ? 1 : 0;
}
