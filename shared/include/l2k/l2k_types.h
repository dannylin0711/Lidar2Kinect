/*
 * Lidar2Kinect shared library - data types.
 *
 * These are the Kinect-for-Windows-v1 ("NUI") concepts the translation layer produces,
 * in a platform-neutral form. They are NOT ABI copies of the NUI structs: the Windows
 * side converts them into NUI_SKELETON_FRAME / NUI_IMAGE_FRAME, the iPad game port
 * consumes them directly. See docs/03-virtual-kinect.md for the conventions.
 *
 * Portable C99, no dependencies, no allocation.
 */
#ifndef L2K_TYPES_H
#define L2K_TYPES_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Kinect v1 skeleton: 20 joints, 6 skeleton slots (same numbers as the NUI SDK). */
#define L2K_JOINT_COUNT 20
#define L2K_SKELETON_SLOTS 6

/* Joint indices. Same order and values as NUI_SKELETON_POSITION_INDEX.
 * LEFT / RIGHT are the PLAYER's left / right (anatomical), as in the NUI SDK. */
enum {
    L2K_JOINT_HIP_CENTER = 0,
    L2K_JOINT_SPINE = 1,
    L2K_JOINT_SHOULDER_CENTER = 2,
    L2K_JOINT_HEAD = 3,
    L2K_JOINT_SHOULDER_LEFT = 4,
    L2K_JOINT_ELBOW_LEFT = 5,
    L2K_JOINT_WRIST_LEFT = 6,
    L2K_JOINT_HAND_LEFT = 7,
    L2K_JOINT_SHOULDER_RIGHT = 8,
    L2K_JOINT_ELBOW_RIGHT = 9,
    L2K_JOINT_WRIST_RIGHT = 10,
    L2K_JOINT_HAND_RIGHT = 11,
    L2K_JOINT_HIP_LEFT = 12,
    L2K_JOINT_KNEE_LEFT = 13,
    L2K_JOINT_ANKLE_LEFT = 14,
    L2K_JOINT_FOOT_LEFT = 15,
    L2K_JOINT_HIP_RIGHT = 16,
    L2K_JOINT_KNEE_RIGHT = 17,
    L2K_JOINT_ANKLE_RIGHT = 18,
    L2K_JOINT_FOOT_RIGHT = 19
};

/* Skeleton slot state. Same values as NUI_SKELETON_TRACKING_STATE. */
enum {
    L2K_SKELETON_NOT_TRACKED = 0,
    L2K_SKELETON_POSITION_ONLY = 1,
    L2K_SKELETON_TRACKED = 2
};

/* Per-joint state. Same values as NUI_SKELETON_POSITION_TRACKING_STATE. */
enum {
    L2K_JOINT_NOT_TRACKED = 0,
    L2K_JOINT_INFERRED = 1,
    L2K_JOINT_TRACKED = 2
};

/* Skeleton quality bits. Same values as NUI_SKELETON_QUALITY_CLIPPED_*. */
enum {
    L2K_QUALITY_CLIPPED_RIGHT = 0x1,
    L2K_QUALITY_CLIPPED_LEFT = 0x2,
    L2K_QUALITY_CLIPPED_TOP = 0x4,
    L2K_QUALITY_CLIPPED_BOTTOM = 0x8
};

/* l2k_skeleton_frame.flags */
enum {
    L2K_FRAME_FLOOR_VALID = 0x1 /* floor_plane holds an estimate (else all zero, like NUI) */
};

/* Where a frame's body data came from (l2k_skeleton_frame.source_kind, l2k_hello.source_kind). */
enum {
    L2K_SOURCE_SYNTHETIC = 0,   /* generated test motion */
    L2K_SOURCE_ARKIT_BODY = 1,  /* ARKit ARBodyAnchor (rear camera, one body) */
    L2K_SOURCE_VISION_3D = 2,   /* Vision 3D body pose */
    L2K_SOURCE_VISION_DEPTH = 3,/* Vision 2D pose lifted with LiDAR depth */
    L2K_SOURCE_REPLAY = 4       /* recorded stream */
};

typedef struct l2k_vec3 {
    float x, y, z;
} l2k_vec3;

typedef struct l2k_vec4 {
    float x, y, z, w;
} l2k_vec4;

/* One skeleton slot, in Kinect skeleton space (metres):
 * origin at the sensor, +z out of the sensor towards the player, +y up (image up),
 * +x to the sensor's left - so a player facing the sensor has their right hand at +x. */
typedef struct l2k_skeleton {
    uint8_t state;        /* L2K_SKELETON_* */
    uint8_t reserved;
    uint16_t quality;     /* L2K_QUALITY_* bits */
    uint32_t tracking_id; /* nonzero and stable while the body stays tracked */
    l2k_vec3 position;    /* body centre (NUI "Position") */
    l2k_vec3 joints[L2K_JOINT_COUNT];
    uint8_t joint_state[L2K_JOINT_COUNT]; /* L2K_JOINT_* */
} l2k_skeleton;

typedef struct l2k_skeleton_frame {
    uint64_t timestamp_us; /* capture time on the sender's monotonic clock, microseconds */
    uint32_t frame_number; /* increases by 1 per produced frame */
    uint32_t flags;        /* L2K_FRAME_* */
    l2k_vec4 floor_plane;  /* (A,B,C,D): A*x+B*y+C*z+D = 0 on the floor, (A,B,C) = unit up, D = sensor height */
    l2k_vec3 up;           /* unit vector opposite to gravity, in skeleton space (NUI vNormalToGravity) */
    uint8_t source_kind;   /* L2K_SOURCE_* */
    uint8_t reserved[3];
    l2k_skeleton skeletons[L2K_SKELETON_SLOTS];
} l2k_skeleton_frame;

/* Virtual Kinect image sizes (what Dance Evolution opens; see docs/01). */
#define L2K_DEPTH_WIDTH 320
#define L2K_DEPTH_HEIGHT 240
#define L2K_COLOR_WIDTH 640
#define L2K_COLOR_HEIGHT 480

/* Depth pixels use the NUI "depth and player index" packing:
 * bits 15..3 = depth in millimetres, bits 2..0 = player index (0 = no player, n = skeleton slot n-1). */
#define L2K_PLAYER_INDEX_BITS 3
#define L2K_PLAYER_INDEX_MASK 0x7
#define L2K_DEPTH_MIN_MM 800
#define L2K_DEPTH_MAX_MM 4000

static inline uint16_t l2k_depth_pack(uint32_t depth_mm, uint32_t player_index) {
    return (uint16_t)((depth_mm << L2K_PLAYER_INDEX_BITS) | (player_index & L2K_PLAYER_INDEX_MASK));
}
static inline uint16_t l2k_depth_mm(uint16_t packed) {
    return (uint16_t)(packed >> L2K_PLAYER_INDEX_BITS);
}
static inline uint8_t l2k_depth_player(uint16_t packed) {
    return (uint8_t)(packed & L2K_PLAYER_INDEX_MASK);
}

#ifdef __cplusplus
}
#endif

#endif /* L2K_TYPES_H */
