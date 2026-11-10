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

/* The body of player `player` (0 or 1) at time t seconds. A 1.7 m figure about 2.5 m from the
 * camera, facing it, swaying and swinging its arms; player 0 stands left of the image centre when
 * `players` is 2, in the centre when it is 1. All joints are reported tracked, hands and toes
 * included. */
void l2k_synth_body(double t, int player, int players, l2k_src_body *out);

#ifdef __cplusplus
}
#endif

#endif /* L2K_SYNTH_H */
