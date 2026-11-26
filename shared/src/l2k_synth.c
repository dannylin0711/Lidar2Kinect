/* Lidar2Kinect - synthetic test motion. */
#include "l2k/l2k_synth.h"

#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846

static l2k_vec3 v3(double x, double y, double z) {
    l2k_vec3 r;
    r.x = (float)x;
    r.y = (float)y;
    r.z = (float)z;
    return r;
}

l2k_vec3 l2k_synth_up(float tilt_deg) {
    /* a camera pitched by tilt sees world up rotated the other way: for a camera looking down
     * (tilt < 0) up leans away from the viewer, towards -z of camera space */
    double a = (double)tilt_deg * PI / 180.0;
    return v3(0.0, cos(a), -sin(a));
}

void l2k_synth_body(double t, int player, int players, l2k_src_body *out) {
    l2k_synth_body_at(t, player, players, L2K_SYNTH_CAMERA_HEIGHT, 0.0f, out);
}

void l2k_synth_body_at(double t, int player, int players, float camera_height, float tilt_deg, l2k_src_body *out) {
    /* level camera space: +x image right, +y up, the figure stands at negative z facing the
     * camera, so its LEFT side is at +x. Floor at y = -camera_height. Rotated by the tilt at the end. */
    double phase = player ? 0.9 : 0.0;
    double cx = (players > 1 ? (player ? 0.65 : -0.65) : 0.0) + 0.20 * sin(2 * PI * (t + phase) / 4.0);
    double cz = -2.5 + 0.20 * sin(2 * PI * (t + phase) / 6.0);
    double floor_y = -(double)camera_height;
    double bob = 0.02 * sin(2 * PI * (t + phase) / 0.5);
    double hip_y = floor_y + 0.90 + bob, neck_y = hip_y + 0.52, head_y = neck_y + 0.17, sh_y = neck_y - 0.03;
    int side, i;

    memset(out, 0, sizeof *out);
    out->tracking_id = (uint32_t)(player + 1);
    out->p[L2K_SRC_PELVIS] = v3(cx, hip_y, cz);
    out->p[L2K_SRC_NECK] = v3(cx, neck_y, cz);
    out->p[L2K_SRC_HEAD] = v3(cx + 0.02 * sin(2 * PI * t / 2.0), head_y, cz);

    for (side = 0; side < 2; side++) {
        double sx = side ? -1.0 : 1.0; /* side 0 = left (+x), 1 = right (-x) */
        int sh = side ? L2K_SRC_SHOULDER_RIGHT : L2K_SRC_SHOULDER_LEFT;
        int el = side ? L2K_SRC_ELBOW_RIGHT : L2K_SRC_ELBOW_LEFT;
        int wr = side ? L2K_SRC_WRIST_RIGHT : L2K_SRC_WRIST_LEFT;
        int ha = side ? L2K_SRC_HAND_RIGHT : L2K_SRC_HAND_LEFT;
        int hp = side ? L2K_SRC_HIP_RIGHT : L2K_SRC_HIP_LEFT;
        int kn = side ? L2K_SRC_KNEE_RIGHT : L2K_SRC_KNEE_LEFT;
        int an = side ? L2K_SRC_ANKLE_RIGHT : L2K_SRC_ANKLE_LEFT;
        int to = side ? L2K_SRC_TOE_RIGHT : L2K_SRC_TOE_LEFT;
        /* arm: angle from hanging down (0) through sideways (pi/2) to straight up (pi) */
        double a = PI / 2 + (PI / 2.2) * sin(2 * PI * (t + phase + side * 1.0) / 2.0);
        double b = a + 0.7 * sin(2 * PI * (t + phase + side * 0.5) / 1.3); /* forearm */
        double shx = cx + sx * 0.19, elx, ely, wrx, wry;
        /* leg: the knee lifts on alternate beats */
        double lift = sin(2 * PI * (t + phase) / 1.0 + side * PI);
        double knee_fwd = lift > 0 ? 0.18 * lift : 0.0, hx = cx + sx * 0.10;
        double ankle_y = floor_y + 0.08 + (lift > 0 ? 0.15 * lift : 0.0);

        elx = shx + sx * 0.28 * sin(a);
        ely = sh_y - 0.28 * cos(a);
        wrx = elx + sx * 0.25 * sin(b);
        wry = ely - 0.25 * cos(b);
        out->p[sh] = v3(shx, sh_y, cz);
        out->p[el] = v3(elx, ely, cz + 0.03);
        out->p[wr] = v3(wrx, wry, cz + 0.08);
        out->p[ha] = v3(wrx + sx * 0.08 * sin(b), wry - 0.08 * cos(b), cz + 0.09);

        out->p[hp] = v3(hx, hip_y - 0.02, cz);
        out->p[kn] = v3(hx, (hip_y + ankle_y) / 2 + 0.01, cz + knee_fwd);
        out->p[an] = v3(hx, ankle_y, cz + 0.02);
        out->p[to] = v3(hx, ankle_y - 0.05, cz + 0.15);

        /* hand tip and thumb: along the forearm past the wrist, the thumb in front */
        out->p[side ? L2K_SRC_HAND_TIP_RIGHT : L2K_SRC_HAND_TIP_LEFT] = v3(wrx + sx * 0.18 * sin(b), wry - 0.18 * cos(b), cz + 0.10);
        out->p[side ? L2K_SRC_THUMB_RIGHT : L2K_SRC_THUMB_LEFT] = v3(wrx + sx * 0.06 * sin(b), wry - 0.06 * cos(b), cz + 0.12);
    }
    {
        /* the face, on a head that turns from side to side (+-20 degrees); the head joint itself does not move with it */
        static const double FACE[5][3] = { /* x = the figure's left, y = up, z = where it faces; from the head centre */
            {0.0, -0.02, 0.085}, {0.032, 0.03, 0.075}, {-0.032, 0.03, 0.075}, {0.075, 0.0, -0.01}, {-0.075, 0.0, -0.01}};
        static const int SRC[5] = {L2K_SRC_NOSE, L2K_SRC_EYE_LEFT, L2K_SRC_EYE_RIGHT, L2K_SRC_EAR_LEFT, L2K_SRC_EAR_RIGHT};
        double yaw = 0.35 * sin(2 * PI * (t + phase) / 3.0), cy = cos(yaw), sy = sin(yaw);
        l2k_vec3 head = out->p[L2K_SRC_HEAD];
        for (i = 0; i < 5; i++) {
            double x = FACE[i][0] * cy + FACE[i][2] * sy, z = -FACE[i][0] * sy + FACE[i][2] * cy;
            out->p[SRC[i]] = v3(head.x + x, head.y + FACE[i][1], head.z + z);
        }
    }
    for (i = 0; i < L2K_SRC_JOINT_COUNT; i++) out->conf[i] = 2;
    if (tilt_deg != 0.0f) {
        /* pitch the camera: camera y axis = (0, cos, sin), camera z axis = (0, -sin, cos) in the level frame */
        double a = (double)tilt_deg * PI / 180.0, c = cos(a), s = sin(a);
        for (i = 0; i < L2K_SRC_JOINT_COUNT; i++) {
            double y = out->p[i].y, z = out->p[i].z;
            out->p[i].y = (float)(y * c + z * s);
            out->p[i].z = (float)(-y * s + z * c);
        }
    }
}
