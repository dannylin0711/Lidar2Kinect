/*
 * Lidar2Kinect shared library - synthetic test motion.
 * A deterministic dancing figure in source camera space, so that senders, receivers and tests on
 * every platform can run without a camera.
 */
#ifndef L2K_SYNTH_H
#define L2K_SYNTH_H

#include "l2k_map.h"

#ifdef __cplusplus
extern "C" {
#endif

/* World up in source camera space for the synthetic scene (a level camera, native landscape). */
#define L2K_SYNTH_UP_X 0.0f
#define L2K_SYNTH_UP_Y 1.0f
#define L2K_SYNTH_UP_Z 0.0f
/* The synthetic camera is this high above the floor (metres). */
#define L2K_SYNTH_CAMERA_HEIGHT 1.0f

/* The sensor placed like Dance Evolution's cabinet Kinect: about 2 m above the floor (Danny,
 * 2026-11-10), pitched down so a player 2.5 m away is centred in the depth image (head at row
 * ~44, feet ~195 of 240). The tilt is chosen for framing, it is not a measured cabinet value.
 * l2k_fake_sender uses this scene. */
#define L2K_SYNTH_CABINET_HEIGHT 2.0f
#define L2K_SYNTH_CABINET_TILT (-24.0f)

/* The body of player `player` (0 or 1) at time t seconds. A 1.7 m figure about 2.5 m from the
 * camera, facing it, swaying and swinging its arms; player 0 stands left of the image centre when
 * `players` is 2, in the centre when it is 1. All joints are reported tracked, hands and toes
 * included. Seen by a level camera L2K_SYNTH_CAMERA_HEIGHT above the floor; world up is
 * L2K_SYNTH_UP_*. */
void l2k_synth_body(double t, int player, int players, l2k_src_body *out);

/* The same figure seen by a camera `camera_height` metres above the floor, pitched by `tilt_deg`
 * (negative = looking down), at the same horizontal distance. tilt 0 and height
 * L2K_SYNTH_CAMERA_HEIGHT give exactly l2k_synth_body. */
void l2k_synth_body_at(double t, int player, int players, float camera_height, float tilt_deg, l2k_src_body *out);
/* World up in source camera space for a camera pitched by `tilt_deg`: (0, cos, -sin). */
l2k_vec3 l2k_synth_up(float tilt_deg);

#ifdef __cplusplus
}
#endif

#endif /* L2K_SYNTH_H */
