/*
 * The vp4u interface between DanceAround's Unity code (VisionPoseForUnity.dll, C#) and the native
 * VisionPoseWrapper.dll it loads, API version 210. Declared from the facts in
 * windows/docs/vp4u-interface.md (read from the game's managed metadata and the original DLL's
 * exports) - not copied from any Konami or NEXT-SYSTEM header.
 *
 * x64 only. All callbacks and table functions are cdecl (= the one x64 convention). Strings are
 * ANSI char* (the C# side marshals them as LPStr). Every C# "bool" return is a 4-byte BOOL.
 */
#ifndef VP4U_ABI_H
#define VP4U_ABI_H

#include <stddef.h>
#include <stdint.h>

#ifndef _M_X64
#error "vp4u is the 64-bit DanceAround interface: build with -A x64"
#endif

#define VP4U_API_VERSION 210
#define VP4U_JOINT_COUNT 30
#define VP4U_MAX_BODIES 16 /* the C# side pre-sizes its lists and arrays to 16 */

/* Joint order: Kinect v2's JointType for 0..24, then five face joints. */
enum {
    VP4U_SPINE_BASE = 0,
    VP4U_SPINE_MID = 1,
    VP4U_NECK = 2,
    VP4U_HEAD = 3,
    VP4U_SHOULDER_LEFT = 4,
    VP4U_ELBOW_LEFT = 5,
    VP4U_WRIST_LEFT = 6,
    VP4U_HAND_LEFT = 7,
    VP4U_SHOULDER_RIGHT = 8,
    VP4U_ELBOW_RIGHT = 9,
    VP4U_WRIST_RIGHT = 10,
    VP4U_HAND_RIGHT = 11,
    VP4U_HIP_LEFT = 12,
    VP4U_KNEE_LEFT = 13,
    VP4U_ANKLE_LEFT = 14,
    VP4U_FOOT_LEFT = 15,
    VP4U_HIP_RIGHT = 16,
    VP4U_KNEE_RIGHT = 17,
    VP4U_ANKLE_RIGHT = 18,
    VP4U_FOOT_RIGHT = 19,
    VP4U_SPINE_SHOULDER = 20,
    VP4U_HAND_TIP_LEFT = 21,
    VP4U_THUMB_LEFT = 22,
    VP4U_HAND_TIP_RIGHT = 23,
    VP4U_THUMB_RIGHT = 24,
    VP4U_NOSE = 25,
    VP4U_EYE_LEFT = 26,
    VP4U_EYE_RIGHT = 27,
    VP4U_EAR_LEFT = 28,
    VP4U_EAR_RIGHT = 29
};

/* Body and joint tracking state. The game treats any value > 0 as valid. */
enum { VP4U_NOT_TRACKED = 0, VP4U_INFERRED = 1, VP4U_TRACKED = 2 };

/* Calibration status reported to the stereo calibration callback (1 = in progress, 3 = success). */
enum { VP4U_CALIB_IN_PROGRESS = 1, VP4U_CALIB_SUCCESS = 3 };

/* An image the wrapper owns; the game AddRefs / Releases it through the table.
 * The test-mode preview builds an RGB24 texture of width x height and loads bytes_per_frame
 * bytes from `pixels`, so bytes_per_frame must be width * height * 3. */
typedef struct vp4u_image_desc {
    void *pixels;
    int32_t width;
    int32_t height;
    int32_t bits_per_pixel;
    int32_t bytes_per_pixel;
    int32_t bytes_per_row;
    int32_t bytes_per_frame;
} vp4u_image_desc;

typedef struct vp4u_joint {
    int32_t state; /* VP4U_NOT_TRACKED / INFERRED / TRACKED */
    float x, y, z; /* metres, see windows/docs/vp4u-interface.md "Coordinates" */
} vp4u_joint;

typedef struct vp4u_body {
    int32_t state; /* > 0: the body is tracked */
    int32_t reserved;
    int64_t tracking_id;
    vp4u_joint joints[VP4U_JOINT_COUNT];
} vp4u_body;

/* Callbacks the game passes in. `ctx` is the pointer it gave with the callback (it passes NULL). */
typedef void (*vp4u_log_cb)(void *ctx, const char *message);
typedef void (*vp4u_video_cb)(void *ctx, uint64_t timestamp_us, const vp4u_image_desc *image);
typedef void (*vp4u_calib_status_cb)(void *ctx, int32_t status, int32_t current, int32_t total, const char *message);
/* main_video == NULL tells the game the stream ended (it then passes null to its own handler). */
typedef void (*vp4u_pose_cb)(void *ctx, uint64_t timestamp_us,
                             const vp4u_body *pose3d, int32_t pose3d_count,
                             const vp4u_body *pose2d, int32_t pose2d_count,
                             const vp4u_body *pose2d_main, int32_t pose2d_main_count,
                             const vp4u_body *pose2d_sub, int32_t pose2d_sub_count,
                             const vp4u_image_desc *main_video, const vp4u_image_desc *sub_video);

/* The table vp4uGetApiTable fills, in this order. */
typedef struct vp4u_api_table {
    uint64_t (*GetCurrentTimestamp)(void); /* microseconds, the clock of every frame timestamp */
    void *(*RegisterLoggerCallback)(void *callback, void *ctx);
    void (*UnregisterLoggerCallback)(void *handle);
    void (*AddRefImageDesc)(vp4u_image_desc *image);
    void (*ReleaseImageDesc)(vp4u_image_desc *image);
    void *(*Initialize)(void); /* non-NULL = loaded */
    void (*Shutdown)(void);
    void (*LoadConfig)(const char *path);
    void (*ReloadConfig)(void);
    int32_t (*OpenRealSense)(void);
    int32_t (*RealSenseLeftIsActive)(void);
    int32_t (*RealSenseRightIsActive)(void);
    int32_t (*TryRebootRealSense)(int32_t hard);
    int32_t (*TryRebuildRealSenseFilters)(void);
    int32_t (*InitVisionPoseRealtime)(const char *product_key); /* the game requires exactly 1 */
    int32_t (*InitVisionPoseOffline)(const char *product_key);  /* the game requires exactly 1 */
    int32_t (*StartRealtimeCameraPreview)(void *left_cb, void *left_ctx, void *right_cb, void *right_ctx);
    void (*StopRealtimeCameraPreview)(void);
    int32_t (*StartStereoCameraCalibration)(void *status_cb, void *preview_cb, void *ctx);
    int32_t (*CommitStereoCameraCalibration)(void);
    void (*CloseStereoCameraCalibration)(void);
    int32_t (*StartRealtimeAnalysis)(void *pose_cb, void *ctx);
    void (*StopRealtimeAnalysis)(void);
    int32_t (*PrepareVideoRecording)(const char *path);
    int32_t (*StartVideoRecording)(void);
    int32_t (*FinalizeVideoRecording)(void);
    int32_t (*DeleteRecordedVideo)(const char *path);
    int32_t (*StartOfflineAnalysis)(const char *path, void *pose_cb, void *ctx);
    void (*StopOfflineAnalysis)(void);
} vp4u_api_table;

/* The DLL's four exports (names as in the original, ordinals 1..4 in this order). */
typedef int32_t (*vp4u_get_api_table_fn)(int32_t version, vp4u_api_table *table, void *log_cb, void *log_ctx);
typedef int32_t (*vp4u_get_version_fn)(void);
typedef int32_t (*vp4u_preboot_fn)(const char *config_path, void *reserved1, void *reserved2);
typedef void (*vp4u_shutdown_fn)(void);

#ifdef __cplusplus
static_assert(sizeof(vp4u_image_desc) == 32, "ImageDesc: pointer + 6 ints");
static_assert(sizeof(vp4u_joint) == 16, "kJointDataSize = 16");
static_assert(offsetof(vp4u_body, tracking_id) == 8, "TrackingState, padding, TrackingId");
static_assert(offsetof(vp4u_body, joints) == 16, "JointData follows TrackingId");
static_assert(sizeof(vp4u_body) == 16 + 480, "JointData is a 480-byte fixed buffer");
static_assert(sizeof(vp4u_api_table) == 29 * 8, "29 function pointers");
#endif

#endif /* VP4U_ABI_H */
