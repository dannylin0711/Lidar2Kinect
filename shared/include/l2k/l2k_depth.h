/*
 * Lidar2Kinect shared library - the virtual Kinect camera and its depth image.
 *
 * The virtual Kinect is a pinhole camera at the phone's camera, with the Kinect v1 depth
 * camera's nominal focal length (285.63 px at 320x240 - the constant the NUI headers use in
 * NuiTransformSkeletonToDepthImage), mirrored like Kinect images are. The colour image is the
 * same camera at twice the resolution, so depth pixel (x,y) <-> colour pixel (2x,2y).
 * See docs/03-virtual-kinect.md.
 */
#ifndef L2K_DEPTH_H
#define L2K_DEPTH_H

#include "l2k_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* NUI_CAMERA_SKELETON_TO_DEPTH_IMAGE_MULTIPLIER_320x240 */
#define L2K_FOCAL_320 285.63f
/* tan of the half field of view: 160/285.63 and 120/285.63 (58.5 x 45.6 degrees) */
#define L2K_TAN_HALF_X (160.0f / L2K_FOCAL_320)
#define L2K_TAN_HALF_Y (120.0f / L2K_FOCAL_320)

/* ---- geometry between the source camera and the virtual Kinect ---- */

/* The source camera's pinhole model, normalised by its image size so it applies to any buffer
 * registered to that image: fxn = fx/width, cxn = cx/width, fyn = fy/height, cyn = cy/height.
 * Image coordinates: origin top-left, x right, y down (ARKit `camera.intrinsics` convention). */
typedef struct l2k_src_camera {
    float fxn, fyn, cxn, cyn;
} l2k_src_camera;

/* "Quadrant": how the device is rolled, in quarter turns, so the virtual Kinect stays upright.
 *   0: the source image's up is up (the sensor's native landscape)   1: the image's right is up
 *   2: the image's down is up (the other landscape)                   3: the image's left is up
 * up_x / up_y: the world up direction in source camera space (x = image right, y = image up). */
int l2k_quadrant_from_up(float up_x, float up_y);

/* Source camera space (ARKit: +x image right, +y image up, +z towards the viewer, i.e. the camera
 * looks along -z) -> Kinect skeleton space, for a point or a direction. */
l2k_vec3 l2k_camera_to_kinect(l2k_vec3 p_camera, int quadrant);

/* Normalised virtual image position (un,vn in 0..1, origin top-left of the mirrored Kinect image)
 * -> normalised source image position (may fall outside 0..1: the source does not see that ray). */
void l2k_virtual_to_source(const l2k_src_camera *cam, int quadrant, float un, float vn, float *sx, float *sy);

/* Skeleton space point -> depth image pixel of a w x h virtual image (NuiTransformSkeletonToDepthImage).
 * Returns 0 if the point is not in front of the sensor. u/v may be outside the image. */
int l2k_skeleton_to_depth_pixel(l2k_vec3 p, int w, int h, float *u, float *v);

/* ---- building the depth + player index image ---- */

typedef struct l2k_depth_source {
    const float *depth_m; /* metres along the optical axis; <= 0 or NaN = unknown. May be NULL. */
    int depth_w, depth_h;
    size_t depth_stride;  /* bytes per row */
    const uint8_t *mask;  /* person segmentation, registered to the same image. May be NULL. */
    int mask_w, mask_h;
    size_t mask_stride;
    uint8_t mask_threshold; /* a pixel is "player" when mask >= threshold */
    uint8_t player_index;   /* 1..6 written on player pixels (single-player sources) */
    uint8_t players_only;   /* nonzero: depth is kept only on player pixels */
    float fallback_depth_m; /* > 0: depth given to player pixels whose depth is unknown (no depth map) */
} l2k_depth_source;

/* Resample the source into the virtual Kinect depth image (out_w x out_h packed uint16).
 * Depth outside L2K_DEPTH_MIN_MM..L2K_DEPTH_MAX_MM becomes 0, like the NUI default range. */
void l2k_depth_render(const l2k_depth_source *src, const l2k_src_camera *cam, int quadrant,
                      uint16_t *out, int out_w, int out_h);

/* Draw every tracked skeleton of `frame` as a capsule figure into a depth + player index image:
 * player index = slot + 1, depth = the limb's depth. The image is cleared first unless `keep` is
 * nonzero; with `keep` the figure is composited over it (a nearer existing player pixel wins).
 * Used by synthetic sources and as the fallback when the device gives no person segmentation. */
void l2k_depth_rasterize(const l2k_skeleton_frame *frame, uint16_t *out, int w, int h, int keep);

/* ---- depth codec "D1" (L2K_DEPTH_D1) ----
 * Byte stream over the pixels in row-major order, `prev` starts at 0:
 *   0x00..0x7F  run:     emit prev (op+1) times
 *   0x80..0xFE  delta:   d = op - 0xBF (-63..+63, never 0); prev += d*8 (depth +d mm, same player); emit once
 *   0xFF lo hi  literal: prev = lo | hi<<8; emit once
 * The stream must produce exactly width*height pixels. */
size_t l2k_depth_d1_bound(size_t pixel_count);
/* Returns the encoded size, or 0 if `cap` is too small. */
size_t l2k_depth_d1_encode(const uint16_t *pixels, size_t pixel_count, uint8_t *out, size_t cap);
/* Returns L2K_OK (0) or a negative value if the stream is malformed. */
int l2k_depth_d1_decode(const uint8_t *in, size_t len, uint16_t *pixels, size_t pixel_count);

#ifdef __cplusplus
}
#endif

#endif /* L2K_DEPTH_H */
