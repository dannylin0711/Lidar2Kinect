/*
 * Lidar2Kinect shared library - body source -> Kinect skeleton frame.
 *
 * A capture source (ARKit, Vision, a replay...) reports each body as a small set of anatomical
 * joints in the source camera's space. l2k_mapper turns that into a Kinect v1 skeleton frame:
 * axis change, the 20 Kinect joints (some derived), tracking states, slots, floor plane.
 * See docs/03-virtual-kinect.md.
 */
#ifndef L2K_MAP_H
#define L2K_MAP_H

#include "l2k_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Source joints. LEFT / RIGHT are the person's own left / right. */
enum {
    L2K_SRC_PELVIS = 0,     /* centre between the hip joints (optional if both hips are given) */
    L2K_SRC_NECK = 1,       /* base of the neck, between the shoulders */
    L2K_SRC_HEAD = 2,       /* centre of the head */
    L2K_SRC_SHOULDER_LEFT = 3,
    L2K_SRC_ELBOW_LEFT = 4,
    L2K_SRC_WRIST_LEFT = 5,
    L2K_SRC_HAND_LEFT = 6,  /* centre of the palm (optional) */
    L2K_SRC_SHOULDER_RIGHT = 7,
    L2K_SRC_ELBOW_RIGHT = 8,
    L2K_SRC_WRIST_RIGHT = 9,
    L2K_SRC_HAND_RIGHT = 10,
    L2K_SRC_HIP_LEFT = 11,  /* head of the femur */
    L2K_SRC_KNEE_LEFT = 12,
    L2K_SRC_ANKLE_LEFT = 13,
    L2K_SRC_TOE_LEFT = 14,  /* ball of the foot (optional) */
    L2K_SRC_HIP_RIGHT = 15,
    L2K_SRC_KNEE_RIGHT = 16,
    L2K_SRC_ANKLE_RIGHT = 17,
    L2K_SRC_TOE_RIGHT = 18,
    L2K_SRC_JOINT_COUNT = 19
};

/* One body from the source, in source camera space: +x image right, +y image up,
 * +z towards the viewer (the camera looks along -z), metres. This is ARKit's camera space. */
typedef struct l2k_src_body {
    uint32_t tracking_id;                   /* stable per person; 0 = let the mapper number it */
    l2k_vec3 p[L2K_SRC_JOINT_COUNT];
    uint8_t conf[L2K_SRC_JOINT_COUNT];      /* 0 = missing, 1 = low confidence / estimated, 2 = tracked */
    /* A missing joint of a tracked body is reported INFERRED, at its last known offset from its
     * parent joint (on the parent if it was never seen): a tracked skeleton never carries a
     * NOT_TRACKED joint. Dance Evolution drops a player whose 20 joint states are not all nonzero. */
} l2k_src_body;

/* Tunables for the joints Kinect places differently from the anatomical source joints.
 * The defaults are first estimates: they are to be calibrated against real Kinect v1 recordings
 * (docs/03 "Calibration"), not facts about the Kinect. */
typedef struct l2k_map_params {
    float hip_center_lift;  /* HIP_CENTER = pelvis + lift * (neck - pelvis) */
    float spine_fraction;   /* SPINE      = pelvis + fraction * (neck - pelvis) */
    float hand_extension;   /* no hand joint: HAND = wrist + extension * (wrist - elbow) */
    float foot_length;      /* no toe joint: FOOT = ankle + length * facing - drop * up (metres) */
    float foot_drop;
    float ankle_height;     /* ankle / toe joint height above the floor, for the floor estimate (metres) */
    float toe_height;
    float clip_margin;      /* a joint beyond margin * half field of view sets a CLIPPED_* bit */
    /* Upper-body mode (see below): proportions used to fill in the lower body, relative to the
     * shoulder width (pelvis, hips) and to the neck-pelvis length (legs). */
    uint8_t upper_body;     /* nonzero: upper-body mode */
    float upper_torso_ratio; /* neck-to-pelvis length = ratio * shoulder width, when the source gives no pelvis */
    float upper_hip_ratio;   /* hip joint to pelvis centre = ratio * shoulder width */
    float upper_leg_ratio;   /* hip-to-ankle length = ratio * neck-to-pelvis length */
    /* Keeping a player through dropouts (see below). */
    float hold_seconds;      /* a body the source loses keeps being reported, frozen, for this long */
    uint8_t keep_identity;   /* nonzero: a body that comes back gets the slot and tracking id it had */
    float identity_seconds;  /* how long an absent player's identity is kept; 0 = for as long as the mapper lives */
    /* Fitting the player into the game's play area (see below). */
    uint8_t fit_play_area;   /* nonzero: move a tracked body that stands outside the play area just inside it */
    float area_near, area_far; /* horizontal distance from the sensor, metres */
    float area_half_width;     /* sideways from the sensor's axis, metres */
    float area_hip_min, area_hip_max; /* hip height above the floor the game is told about, metres */
    /* Placing the players on the game's own standing points (see "Normalising" below). Replaces the fit when on. */
    uint8_t normalize;         /* nonzero: on */
    uint8_t norm_players;      /* 1 or 2: how many players the game is told about */
    float norm_distance;       /* metres in front of the sensor: 2.5 */
    float norm_spacing;        /* two players: each this far either side of the middle: 0.61 */
    float norm_follow_seconds; /* how quickly the standing point follows the player; 0 = the hips are pinned */
    float norm_reach;          /* how far from the standing point a player may be reported at most, metres */
} l2k_map_params;

/* The play area. Dance Evolution only takes a body as a player while its position lies in a box in
 * front of the sensor: 1.80 to 3.20 m away, 1.25 m either side, 0 to 2 m above the floor it is told
 * about - with 5 cm of slack to be taken on and 20 cm to be kept (read from gamekdm.dll and run under
 * an emulator, docs/01 section 8). A phone with a wide lens invites standing closer than that, and a
 * player who drops out while standing in the 15 cm band between the two margins cannot get back in.
 * The game's judging does not use where the player stands (it maps limb directions onto the dancer),
 * so with `fit_play_area` the mapper slides a tracked body that is outside the area horizontally to
 * just inside it (10 cm in from the edges), and makes the floor it reports agree with the body
 * (the hip between area_hip_min and area_hip_max above it). A body already inside is not touched.
 * The depth and colour images are not changed: they show where the player really is. */

/* Dropouts. A camera-based source loses a body far more easily than a Kinect does (one missed
 * detection is enough), and Dance Evolution follows its player by tracking id: a player who comes
 * back under a new id is a stranger to it. So the mapper, not the source, owns the identities:
 *   - the tracking ids it reports are its own (1, 2, ...), not the source's;
 *   - a body missing for up to `hold_seconds` is still reported, in its slot, with its last
 *     skeleton and every joint INFERRED - a short gap is invisible to the game;
 *   - after that the slot is empty, but with `keep_identity` its slot and tracking id stay
 *     reserved, and the next body the source reports takes over the reserved identity nearest to
 *     where it stands. With `keep_identity` off the identity is dropped once the hold ends, and a
 *     returning body gets a new id, as on a Kinect. */

/* Upper-body mode, for a camera that only sees the player from about the waist up.
 *   - A body is tracked from its neck and both shoulders; a pelvis is not required (when the source
 *     has none, it is placed straight below the neck).
 *   - The legs are never taken from the source: hips, knees, ankles and feet are a standing pose
 *     straight down from the pelvis, reported INFERRED, so the skeleton stays complete.
 *   - Every frame carries L2K_FRAME_UPPER_BODY, so a receiver knows the legs are made up.
 * The arms, head and shoulders are the player's own. Anything a game judges on the legs is judged
 * against the standing pose: the layer does not fake leg movement. */

void l2k_map_params_default(l2k_map_params *params);

/* Normalising. Dance Evolution places its world 2.5 m in front of the sensor (docs/01 section 7): that is the
 * middle of the box a player must stand in, and where its dancer stands. Its two-player rule expects the players
 * 0.61 m either side of the middle. With `normalize` the mapper reports, whatever the room looks like:
 *  - a level sensor: every joint is turned about the sensor so that "up" is the y axis (the game no longer has to
 *    undo the tilt from the elevation angle), and the floor plane is (0, 1, 0, sensor height);
 *  - each player on a standing point: one player at (0, 2.5 m); two players at -0.61 m and +0.61 m, each on the
 *    side they really stand on. The standing point follows the player's own position slowly
 *    (`norm_follow_seconds`), so steps and leans are reported as they are and only the place in the room is removed;
 *    a player is never reported more than `norm_reach` from the standing point, i.e. always inside the play area;
 *  - only `norm_players` players: those who were there first. Other bodies are not sent.
 * The depth and colour images are not changed. `picture` keeps every skeleton as it was before this (and before the
 * fit), for drawing over the camera picture. */
typedef struct l2k_mapper {
    l2k_map_params params;
    uint32_t frame_number;
    uint32_t slot_id[L2K_SKELETON_SLOTS]; /* source id bound to each slot now, 0 = none */
    uint32_t out_id[L2K_SKELETON_SLOTS];  /* the tracking id each slot reports, 0 = the slot has no identity */
    uint32_t next_out_id;
    uint64_t last_seen_us[L2K_SKELETON_SLOTS]; /* when the slot's body was last tracked */
    uint8_t has_last[L2K_SKELETON_SLOTS];
    l2k_skeleton last[L2K_SKELETON_SLOTS];     /* its last tracked skeleton */
    float sensor_height;                  /* metres above the floor */
    int floor_valid;
    int floor_fixed;
    /* per slot: the skeleton of the frame last mapped before the play-area fit / normalising moved it: where the
     * body is in the camera picture. A local preview draws these over the picture. */
    l2k_skeleton picture[L2K_SKELETON_SLOTS];
    /* normalising: per slot the standing point's place in the room (level frame), and the side given to it */
    struct {
        float x, z;
        uint8_t valid;
        int8_t side; /* two players: -1 / +1, 0 = not decided */
    } anchor[L2K_SKELETON_SLOTS];
    uint64_t norm_last_us;
    /* per slot: each joint's last known offset from its parent joint, used while the source loses it */
    struct {
        l2k_vec3 offset[L2K_JOINT_COUNT];
        uint8_t valid[L2K_JOINT_COUNT];
    } memory[L2K_SKELETON_SLOTS];
} l2k_mapper;

/* params may be NULL for the defaults. */
void l2k_mapper_init(l2k_mapper *m, const l2k_map_params *params);

/* Give the mapper a measured sensor height above the floor (e.g. from plane detection).
 * height <= 0 returns to estimating it from the feet. */
void l2k_mapper_set_floor(l2k_mapper *m, float sensor_height_m);

/* Build the next skeleton frame.
 *   up_camera : world up (opposite to gravity) in source camera space
 *   quadrant  : device roll in quarter turns, see l2k_quadrant_from_up() in l2k_depth.h */
void l2k_mapper_map(l2k_mapper *m, const l2k_src_body *bodies, int body_count, l2k_vec3 up_camera,
                    int quadrant, uint64_t timestamp_us, uint8_t source_kind, l2k_skeleton_frame *out);

/* Sensor tilt in degrees from a frame's `up` vector: positive = looking upwards
 * (what NuiCameraElevationGetAngle reports). */
float l2k_elevation_degrees(l2k_vec3 up);

/* ---- joint smoothing (NuiTransformSmooth equivalent) ----
 * Holt double exponential filter as published in Microsoft's "Skeletal Joint Smoothing" white
 * paper sample. Parameter names and meaning follow NUI_TRANSFORM_SMOOTH_PARAMETERS. Whether its
 * output matches Kinect10.dll's NuiTransformSmooth exactly is NOT verified (docs/01 "Open"). */
typedef struct l2k_smooth_params {
    float smoothing;            /* 0..1 */
    float correction;           /* 0..1 */
    float prediction;           /* frames */
    float jitter_radius;        /* metres */
    float max_deviation_radius; /* metres */
} l2k_smooth_params;

typedef struct l2k_smoother {
    struct {
        uint32_t tracking_id;
        struct {
            l2k_vec3 raw, filtered, trend;
            uint32_t count;
        } joint[L2K_JOINT_COUNT];
    } slot[L2K_SKELETON_SLOTS];
} l2k_smoother;

/* The NUI defaults (also what Dance Evolution passes): 0.5, 0.5, 0.5, 0.05, 0.04. */
void l2k_smooth_params_default(l2k_smooth_params *params);
void l2k_smoother_init(l2k_smoother *s);
/* Filters the joints of every tracked skeleton of `frame` in place. */
void l2k_smoother_apply(l2k_smoother *s, l2k_skeleton_frame *frame, const l2k_smooth_params *params);

#ifdef __cplusplus
}
#endif

#endif /* L2K_MAP_H */
